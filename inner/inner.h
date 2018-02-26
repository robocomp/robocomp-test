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

#include <unordered_map>

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
		Proxy(const std::shared_ptr<T> &node_, const std::shared_ptr<Inner> &inner) : T(node_->getId(), inner)
		{
			node = node_;
			if (node->lock())
				std::cout << node->getId() << " dentro " << node->getId2() << std::endl;
			else
				throw std::runtime_error("Could not lock the mutex");
		}
		~Proxy()										{ std::cout << "me matan" << std::endl;  node->unlock(); }
		
		//std::shared_ptr<T> operator ->() const	{ return node;}
		
		std::string getId() const 						{ return node->getId(); }
		std::string getId2() const 						{ return node->getId2(); }
		void setId(const std::string &id_) 				{ node->setId(id_);}
		void setId2(const std::string &id_) 			{ node->setId(id_);}
		void addChild(NODE *node_)						{ node->children.push_back(node_);};
		void print() const								{ node->print(); }								
		void lock() 									{ node->lock();}
		void unlock()   								{ node->unlock();}
		
	private:		
		std::shared_ptr<T> node;
};

class Inner
{
	
	public:
		Inner(){};
		////////////////////////////////
		/// Factory constructor
		///////////////////////////////
		template<typename T, typename... Ts>
		std::shared_ptr<T> newNode(Ts&&... params)
		{
			auto t = std::make_tuple(params...);
			std::string id = (std::get<0>(t));
			std::shared_ptr<T> node(new T(std::forward<Ts>(params)...)); 
			bool ok = hash.insert({id, std::static_pointer_cast<NODE>(node)}).second;
			if(ok)
				return node;
			else 
				throw std::out_of_range("Cannot insert id in hash");
		}
		//////////////////////////////////////
		/// Node getter
		/////////////////////////////////////
		template <typename N> 
		std::shared_ptr<Proxy<N>> getNode(const std::string &id) 
		{
			try 
			{ 
				auto n = hash.at(id);
				auto nn = std::static_pointer_cast<N>(n);
				return std::shared_ptr<Proxy<N>>(new Proxy<N>(nn, std::shared_ptr<Inner>()));
			}
			catch(const std::exception &e)
			{ 
				std::cout << e.what() << std::endl;
				return std::shared_ptr<Proxy<N>>();
			}
		}
		void setRoot(const TRANSFORMPtr &r)						{ root = r;};
		void print() const 										{ root->print();}
		
		std::unordered_map<std::string, std::shared_ptr<NODE>> hash;
		
	private:
		TRANSFORMPtr root;
};

#endif // INNER_H
