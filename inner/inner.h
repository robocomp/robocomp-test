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

template <typename T>
class Proxy 
{
	public:
		Proxy(const std::shared_ptr<T> &node_)
		{
			node = node_;
			//node->lock();
			std::cout << node->getId() << " dentro " << node->getId2() << std::endl;
		}
		~Proxy()								{ std::cout << "me matan" << std::endl;  /*node->unlock(); */}
		
		std::shared_ptr<T> * operator ->() const	{ return node;}
		//std::string getId() const 			{ std::cout << "En getId del proxy" << std::endl; return node->getId(); }
		//std::string getId2T() const 			{ return node->getId2(); }
		
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
				throw;
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
				return std::shared_ptr<Proxy<N>>(new Proxy<N>(nn));
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
