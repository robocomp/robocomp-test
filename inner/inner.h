/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INNER_H
#define INNER_H

#include <string>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <stack>
#include <unordered_map>
#include <iomanip>      // std::setfill, std::setw

#include "safe_ptr.h"
#include "nodes.h"

using NODEPtr = std::shared_ptr<NODE>;
using TRANSFORMPtr = std::shared_ptr<TRANSFORM>;
using JOINTPtr = std::shared_ptr<JOINT>;

using namespace std::chrono_literals;

////////////////////////////////////
/// Proxy class. Have to reproduce Node's interface 'cause -> overload does not work
////////////////////////////////////

template <typename T>
class Proxy : public T
{
	public:
		Proxy(const std::shared_ptr<T> &node_, const std::shared_ptr<Inner> &inner) : T()
		{
			node_->lock();
			node = node_;
			node->incWaiting();
		}
		~Proxy()
		{
			node->decWaiting();
			node->unlock();
		}

		std::shared_ptr<T> operator ->() const	{ return node;}

		std::string getId() const 							{ return node->getId();}
		void setId(const std::string &id_) 					{ node->setId(id_);}
		void addChild(const std::string id_)				{ node->addChild(id_);};
		void removeChild(const std::string &childId)		{ node->removeChild(childId);};
		bool addChildToParent(const std::string &parentId)	{ return node->addChildToParent(parentId);};
		std::string getParentId() const						{ return node->getParentId();};
		std::string getChildId(unsigned int i) const		{ return node->getChildId(i);};
		std::vector<std::string> getChildren() const		{ return node->getChildren();};
		size_t getNumChildren() const 						{ return node->getNumChildren();};
		void print() const									{ node->print(); }
        void lock() 					                   	{ node->lock();}
		void unlock()   				                  	{ node->unlock();}
		bool isMarkedForDelete() const						{ return node->markedForDelete(); };
		void markForDelete() 								{ node->markForDelete();  };
		void incWaiting() 									{ node->incWaiting();}
		void decWaiting() 									{ node->decWaiting();}
		ulong getWaiting() const 							{ return node->getWaiting();}
		friend std::ostream& operator<< (std::ostream &out, const std::shared_ptr<Proxy<NODE>> &node);

	protected:
		std::shared_ptr<T> node;
};

template <typename K, typename V>
class ThreadSafeHash
{
	using myMap = std::unordered_map<K, V>;

	public:
		ThreadSafeHash(){};
		ThreadSafeHash(const ThreadSafeHash &other) = delete;
		ThreadSafeHash& operator=(const ThreadSafeHash &other) = delete;

		V at(const K &key)	const
		{ //std::shared_lock<std::shared_mutex> lock(mymutex);
			return hash.at(key);

		};

        auto insert(std::pair<K, V> data)
		{
			//std::unique_lock<std::shared_mutex> lock(mymutex);
			auto t = hash.insert(data);
			return t;
		}

		void erase(const K &key)
		{
            //std::unique_lock<std::shared_mutex> lock(mymutex);
			hash.erase(key);
		}
		auto size() const
		{ //std::shared_lock<std::shared_mutex> lock(mymutex);
			return hash.size();
		}

		std::vector<typename myMap::key_type> keys()
		{
			//std::shared_lock<std::shared_mutex> lock(mymutex);
			std::vector<typename myMap::key_type> r;
			r.reserve(hash.size());
			for(auto&& kvp : hash)
			{
				r.push_back(kvp.first);
			}
			return r;
		}

		std::vector<typename myMap::mapped_type> values()
		{
			//std::shared_lock<std::shared_mutex> lock(mymutex);
			std::vector<typename myMap::mapped_type> r;
			r.reserve(hash.size());
			for (auto&& kvp : hash)
			{
				r.push_back(kvp.second);
			}
			return r;
		}

	private:
		myMap hash;
		//mutable std::shared_mutex mymutex;
};

class Inner
{

	public:
        Inner(){};
		Inner(const Inner *inner)
        {
            rootid = inner->rootid;
            rootptr = inner->rootptr;
        };
		////////////////////////////////
		/// Factory constructor
		///////////////////////////////
		template<typename T, typename... Ts>
		std::shared_ptr<T> newNode(Ts&&... params)
		{
            std::unique_lock<std::mutex>(nn_mutex);
            auto t = std::make_tuple(params...);
			std::string id = (std::get<0>(t));
            try
			{
                std::shared_ptr<T> node(new T(std::forward<Ts>(params)...));
                bool ok = hash->insert({id, std::static_pointer_cast<NODE>(node)}).second;
                if(ok)
                    return node;
                else
                    throw std::out_of_range("Cannot insert id in hash");
            }
			catch(const std::exception &e)
            { throw std::runtime_error("Cannot insert node with invalid parent"); }
		}
		//////////////////////////////////////
		/// Node getter
		/////////////////////////////////////
		template <typename N>
		std::shared_ptr<Proxy<N>> getNode(const std::string &id)
        {
            std::unique_lock<std::mutex>(gn_mutex);
            try
            {
                auto n = hash->at(id);
                if(n->isMarkedForDelete())
                    return std::shared_ptr<Proxy<N>>(nullptr);
                auto nn = std::static_pointer_cast<N>(n);
                return std::shared_ptr<Proxy<N>>(new Proxy<N>(nn, std::shared_ptr<Inner>()));
            }
            catch(const std::exception &e)
            {
                std::cout << e.what() << " for ID: " << id << std::endl;
                return std::shared_ptr<Proxy<N>>(nullptr);
            }
        }


		void setRootId(const std::string &r);
		std::string getRootId() const 				{return(rootid);};
		void print() const;
		void printIter();
		void deleteNode(const std::string &id);
                void removeSubTree(const std::string id, std::vector<std::string> &l);

		//ThreadSafeHash<std::string, NODEPtr> hash;
		sf::safe_ptr<ThreadSafeHash<std::string, NODEPtr>> hash;
	private:
		std::string rootid;
		TRANSFORM *rootptr;
        mutable std::mutex gn_mutex, nn_mutex;
};

#endif // INNER_H
