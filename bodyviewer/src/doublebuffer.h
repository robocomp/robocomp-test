//
// Created by robolab on 24/07/18.
//

#ifndef PROJECT_DOUBLEBUFFER_H
#define PROJECT_DOUBLEBUFFER_H
#include <mutex>
#include <iostream>
#include <queue>
#include <cassert>

template <typename I, typename O> class Converter
{
    public:
        virtual bool ItoO( I & iTypeData, O &oTypeData)=0;
        virtual bool OtoI(const O & oTypeData, I &iTypeData)=0;
		virtual bool clear(O & oTypeData)=0;
};

// Default converter. Only works for generic containers
template <typename I, typename O> class ConverterDefault : Converter<I,O>
{
    public:
        bool ItoO( I & iTypeData, O &oTypeData) 
        { 
            if(sizeof(iTypeData) == sizeof(oTypeData)) 
            {   std::swap(iTypeData, oTypeData); 
                return true;
            } else
                return false;
        }
        bool OtoI(const O & oTypeData, I &iTypeData){ return true;};
        bool clear(O & oTypeData){ return true;};
};

template <typename I, typename O, typename C=ConverterDefault<I,O>> class DoubleBuffer
{
private:
    mutable std::mutex bufferMutex;
    C converter;
    std::queue<O> queue;
    std::atomic<uint> MAX_QUEUE_SIZE;

public:
    DoubleBuffer()
    {
        MAX_QUEUE_SIZE.store(10);
    };
    
	void clear()
	{
    	// this->to_clear = true;
		// if( converter->clear(writeBuffer))
		// {
		// 	std::lock_guard<std::mutex> lock(bufferMutex);
		// 	std::swap(writeBuffer,readBuffer);
		// }
	}

    bool isEmpty()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return queue.empty();
    }
    int size()
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        return queue.size();
    }
    void setMaxQueueSize(const uint newMax)
    {
        MAX_QUEUE_SIZE.store(newMax);
    }

    O get() 
    {
        std::lock_guard<std::mutex> lock(bufferMutex);
        if(queue.empty() == false)
        {
            O oData = queue.front();
            queue.pop();
//            qDebug() << "read " << queue.size();
            return oData;
        }
        else 
            return O();
    }

    void put(I &d)
    {
        O buffer;
        if( converter.ItoO(d, buffer))
        {
            std::lock_guard<std::mutex> lock(bufferMutex);
            if(queue.size() < MAX_QUEUE_SIZE.load())
                queue.emplace(std::move(buffer));
            else
                queue.back() = buffer;
//            qDebug() << "in put queue size " << queue.size();
        }
    }
};

#endif //PROJECT_DOUBLEBUFFER_H


