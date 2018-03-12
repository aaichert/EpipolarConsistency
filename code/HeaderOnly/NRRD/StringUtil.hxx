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

#ifndef __string_util_hxx
#define __string_util_hxx

#include "StringConversion.hxx"
#include "StringType.hxx"

#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>

/// Test a string for a prefix
inline bool hasPrefix(const std::string& str, const std::string& prefix)
{
	if (str.length()<prefix.length()) return false;
	return std::mismatch(prefix.begin(), prefix.end(), str.begin()).first == prefix.end();
}

/// Right trim
inline void rtrim(std::string &str , const std::string& t = " \t")
{
	str.erase(str.find_last_not_of(t)+1);
}

/// Left trim
inline void ltrim(std::string& str, const std::string& t = " \t")
{
	str.erase(0,str.find_first_not_of(t));
}

/// Trim
inline void trim(std::string& str, const std::string& t = " \t")
{
	rtrim(str,t);
	ltrim(str,t);
} 

/// Remove the part right of last occurence of delim and return it
inline std::string splitRight(std::string& str, const std::string& delim)
{
	std::string::size_type loc=str.find_last_of(delim);
	std::string right;
	if (loc!=std::string::npos)
	{
		right=str.substr(loc+1,std::string::npos);
		str=str.substr(0,loc); // left
	}
	else
	{
		right=str;
		str.clear();
	}
	return right;
}

/// Remove the part left of first occurence of delim and return it
inline std::string splitLeft(std::string& str, const std::string& delim)
{
	std::string::size_type loc=str.find_first_of(delim);
	std::string left;
	if (loc!=std::string::npos)
	{
		left=str.substr(0,loc);
		str=str.substr(loc+1,std::string::npos);
	}
	else
	{
		left=str;
		str.clear();
	}
	return left;
}

/// write text to file
inline bool fileWriteString(const std::string& filename, const std::string& contents)
{
	std::ofstream file(filename.c_str());
	if (!file) return false;
	file << contents;
	file.close();
	return true;
}

/// Read a complete text file
inline std::string fileReadString(const std::string filename)
{
	std::ifstream file(filename.c_str());
	if (!file.is_open() && file.good())
		return "";
	std::string all;
	getline(file,all,'\0');
	file.close();
	return all;
}

/// Make all lower-case no whitespace strings
inline void normalize(std::string& name)
{
	std::transform(name.begin(),name.end(),name.begin(),tolower);
	std::replace(name.begin(),name.end(),' ','-');
	std::replace(name.begin(),name.end(),'/','-');
}

namespace EscapeSequence {

	/// Escape a list of characters in input string with escape sequence. By default uses typical backslash-escape.
	inline std::string escape(const std::string& input, const std::map<unsigned char,unsigned char>& replace_characters, char escape_character)
	{
		std::string ret;
		for (auto it=input.begin();it!=input.end();++it) {
			auto replace=replace_characters.find(*it);
			if (replace!=replace_characters.end()) ret+=std::string(1,escape_character)+std::string(1,replace->second);
			else if (*it==escape_character) ret+= std::string(1,escape_character) + std::string(1,escape_character);
			else ret+=std::string(1,*it);
		}
		return ret;
	}

	/// Removes escape sequence from a string. Invalid escape sequences expand to nothing, i.e. "\x" -> "" for unknown x and backslash escape character.
	inline std::string unescape(const std::string& input, const std::map<unsigned char,unsigned char>& replace_characters, char escape_character)
	{
		std::string ret;
		for (auto it=input.begin();it!=input.end();++it)
			if (*it==escape_character)
			{
				if (++it==input.end()) break;
				auto cit=replace_characters.find(*it);
				if (cit!=replace_characters.end()) ret += std::string(1,cit->second);
				else if (*it==escape_character) ret += std::string(1,escape_character);
				// else: invalid escape sequence, ignored.
			}
			else ret += std::string(1,*it);
		return ret;
	}

	/// Newlines are replaced by "\n"
	inline std::string escape_newline(const std::string& input) {
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['\n']='n';
		return escape(input,escape_characters,'\\');
	}

	/// "\n" is replaced by newline character.
	inline std::string unescape_newline(const std::string& input) {
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['n']='\n';
		return unescape(input,escape_characters,'\\');
	}

	/// whitespace to '_', underscore to "^u" double quotes to "^q", newlines to "^n", equals sign to "^e" and '^' to "^^".
	inline std::string escape_attrib_key(const std::string& input) {
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['\"']='q';
		escape_characters['\n']='n';
		escape_characters[ '=']='e';
		escape_characters[ '_']='u';
		std::string tmp=escape(input,escape_characters,'^');
		std::replace(tmp.begin(),tmp.end(),' ','_');
		return tmp;
	}

	/// Reverse of escape_attrib_key(...).
	inline std::string unescape_attrib_key(const std::string& input) {
		std::string tmp=input;
		std::replace(tmp.begin(),tmp.end(),'_',' ');
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['q']='\"';
		escape_characters['n']='\n';
		escape_characters['e']='=';
		escape_characters['u']='_';
		return unescape(tmp,escape_characters,'^');
	}

	/// double quotes to "^q", newlines to "^n" and '^' to "^^".
	inline std::string escape_attrib_value(const std::string& input) {
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['\"']='q';
		escape_characters['\n']='n';
		return escape(input,escape_characters,'^');
	}
	/// "^q" to ouble quote, "^n" to newlines and "^^" to '^'
	inline std::string unescape_attrib_value(const std::string& input) {
		std::map<unsigned char,unsigned char> escape_characters;
		escape_characters['q']='\"';
		escape_characters['n']='\n';
		return unescape(input,escape_characters,'^');
	}

} // namespace EscapeSequence


/// Parse XML-Style attributes into an std::map of strings. Attributes are added to the std::map of strings out.
inline void attrib_parse(const std::string& in, std::map<std::string,std::string>& out)
{
	using namespace EscapeSequence;
	size_t pos=0;
	std::string key,value;
	for (;;)
	{
		size_t next=in.find("=",pos);
		size_t v1=in.find("\"",next);
		size_t v2=in.find("\"",v1+1);
		if (next==std::string::npos||v1==std::string::npos||v2==std::string::npos)
			return;
		if (v2!=std::string::npos)
		{
			key=in.substr(pos,next-pos);
			value=in.substr(v1+1,v2-v1-1);
			trim(key);
			trim(value);
			key=unescape_attrib_key(key);
			value=unescape_attrib_value(value);
			out[key]=value;
			pos=v2+1;
		}
	}
}

/// Create a string with XML-Style attributes from a std::map of strings
inline std::string attrib_list(const std::map<std::string,std::string>& in)
{
	using namespace EscapeSequence;
	std::string ret;
	for (auto it=in.begin();it!=in.end();++it)
		ret += escape_attrib_key(it->first) + "=\""+escape_attrib_value(it->second)+"\" ";
	return ret;
}


#endif // __string_util_hxx
