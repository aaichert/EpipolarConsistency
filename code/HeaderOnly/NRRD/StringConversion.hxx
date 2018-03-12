//
//  Library: GetSet
//  c++ library for load/saving *typed* and *named* properties and automatic GUI.
//  
//  Copyright (c) by André Aichert (aaichert@gmail.com)
//    
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//  
//    http://www.apache.org/licenses/LICENSE-2.0
//    
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef __string_conversion_hxx
#define __string_conversion_hxx

#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>
#include <set>

/// Reset value (call constructor or zeros c-types, see specializations)
template <typename T> inline T default_value() { T v; return v; }

// Type general conversion to string
template <typename T> inline std::string toString(const T& in)
{
	std::ostringstream strstr;
	strstr << in;
	return strstr.str();
}

// General conversion from string to another type
template <typename T> inline T stringTo(const std::string& in)
{
	T value=default_value<T>();
	std::istringstream strstr(in);
	strstr >> value;
	return value;
}

template <> inline std::string toString<>(const std::string& in) { return in; }
template <> inline std::string stringTo<>(const std::string& in) { return in; }

template <> inline std::string toString<>(const bool& in) { return in ? "true" : "false"; }
template <> inline bool stringTo<>(const std::string& in)
{
	std::string s=in;
	s.erase(s.find_last_not_of(" \t")+1);
	s.erase(0,s.find_first_not_of(" \t"));
	transform(s.begin(), s.end(), s.begin(), ::tolower);
	if (s=="" || s=="false" || s=="no") return false;
	if (s=="true" || s=="yes") return true;
	return stringTo<int>(in)>0;
}

// Conversion from vector of any type to string
template <typename T> inline std::string vectorToString(const std::vector<T>& in, const std::string& delim=" ")
{
	if (in.empty()) return std::string();
    typename std::vector<T>::const_iterator it=in.begin();
	std::string ret=toString(*it);
	for (++it;it!=in.end();++it)
		ret+=delim+toString(*it);
	return ret;
}

// Conversion of a string to a vector of any type
template <typename T=std::string> inline std::vector<T> stringToVector(const std::string& in, const char delim=' ', bool multiple=false)
{
	std::string item;
	std::vector<T> ret;
	std::istringstream str(in);
	for (;std::getline(str,item,delim);str&&!str.eof())
		if (multiple||!item.empty())
		ret.push_back(stringTo<T>(item));
	return ret;
}

// Conversion from set of any type to string
template <typename T> inline std::string setToString(const std::set<T>& in, const std::string& delim=" ")
{
	if (in.empty()) return std::string();
    typename std::set<T>::const_iterator it=in.begin();
	std::string ret=toString(*it);
	for (++it;it!=in.end();++it)
		ret+=delim+toString(*it);
	return ret;
}

// Conversion of a string to a set of any type
template <typename T=std::string> inline std::set<T> stringToSet(const std::string& in, const char delim=' ')
{
	std::string item;
	std::set<T> ret;
	std::istringstream str(in);
	for (;std::getline(str,item,delim);str&&!str.eof())
		ret.insert(stringTo<T>(item));
	return ret;
}

// Specializations of toString and stringTo for select vector-types assuming seperating semicolon
#define _DEFINE_TYPE(TYPE) \
	template <> inline std::string toString<>(const std::vector<TYPE>& in) {return vectorToString(in,";");} \
	template <> inline std::vector<TYPE> stringTo<>(const std::string& in) {return stringToVector<TYPE>(in,';');}
_DEFINE_TYPE(std::string)
#include "BaseTypes.hxx"

// Specializations
#define _DEFINE_TYPE(X)  template<> inline X default_value<X>() { return 0; }
#include "BaseTypes.hxx"

/// Overload of string conversion for specific lengths
template <typename T> inline std::string toString(T in, int width, char fill='0')
{
	std::ostringstream strstr;
	strstr << std::setfill(fill) << std::setw(width) << in;
	return strstr.str();
}


#endif // __string_conversion_hxx
