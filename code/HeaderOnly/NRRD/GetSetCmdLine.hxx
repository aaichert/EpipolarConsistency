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

#ifndef __GetSetCmdLine_hxx
#define __GetSetCmdLine_hxx

#include "GetSetMinimal.hxx"
#include <set>

namespace GetSetIO {
	/// Set properties via command line.
	class CmdLineParser {
	public:
		typedef std::map<std::string, std::string>	MapStrStr;

		CmdLineParser(GetSetDictionary& d=GetSetDictionary::global())
			: dict(d)
		{}

		/// Parse. Returns true if all required flags have been found. There may still be unhandled args.
		bool parse(int argc, char **argv);

		// FIXME just print a list of all registered command line flags
		std::string getSynopsis() const;

		/// Unhandled command line arguments
		const std::map<int,std::string>& getUnhandledArgs() const {return unhandledArgs;}

		/// Declare flag(s) automatically or explicitly if flag is supplied.
		CmdLineParser& declare(const std::string& path="", const std::string& flag="");

		/// The key supplied is required and MUST be defined for parse(...) to return true.
		CmdLineParser& require(const std::string& key="", const std::string& flag="") { declare(key,flag); required.insert(key); return *this; }

		/// Define an unnamed command line argument.
		CmdLineParser& index(const std::string& key, int idx, bool required=true)
		{
			declare(key, toString(idx));
			if (required) require(key);
			return *this;
		}

		/// After parse() this is how to check if an argument was specified
		bool isSpecified(const std::string& path) const
		{
			return found.find(path)!=found.end();
		}

	protected:
		/// Stores all Flags (and numbers for indexed params) together with a key
		MapStrStr flags;

		/// Unhandled command line arguments by its index in argv[]
		std::map<int,std::string> unhandledArgs;
	
		/// Set of required parameters (removed if found) and found parameters
		std::set<std::string> required, found;

		/// Reference to the collection of parameters
		GetSetDictionary& dict;
	};

	/////////////////////////////////////
	// Implementations
	/////////////////////////////////////

	std::string CmdLineParser::getSynopsis() const
	{
		std::map<std::string,std::set<std::string> > args;
		std::ostringstream str;
		for (MapStrStr::const_iterator it=flags.begin();it!=flags.end();++it)
			args[it->second].insert(it->first);
		// Required parameters first
		if (!required.empty())
		{
			str << "---------------------------\n";
			str << "Required command line args:\n\n";
			for (std::map<std::string,std::set<std::string> >::iterator it=args.begin();it!=args.end();++it)
				if (required.find(it->first)!=required.end())
				{
					str << "Parameter " << it->first << "\n";
					std::set<std::string>::iterator f=it->second.begin();
					str << ((*f)[0]=='-' ? *f : std::string("argument #")+*f);
					for (++f;f!=it->second.end();++f)
						str << " or " << ((*f)[0]=='-' ? *f : std::string("argument #")+*f);
					str << std::endl;
					std::string d=GetSet<>(it->first).getDescription();
					if (!d.empty())
						str << d << std::endl;
					str << std::endl;
				}
			str << "---------------------------\n\n";
		}
		// Optional parameters second
		for (std::map<std::string,std::set<std::string> >::iterator it=args.begin();it!=args.end();++it)
			if (required.find(it->first)==required.end())
			{
				str << "Parameter " << it->first << "\n";
				std::set<std::string>::iterator f=it->second.begin();
				str << ((*f)[0]=='-' ? *f : std::string("argument #")+*f);
				for (++f;f!=it->second.end();++f)
					str << " or " << ((*f)[0]=='-' ? *f : std::string("argument #")+*f);
				str << std::endl;
				std::string d=GetSet<>(it->first).getDescription();
				if (!d.empty())
					str << d << std::endl;
				str << std::endl;
			}
		return str.str();
	}

	CmdLineParser& CmdLineParser::declare(const std::string& path, const std::string& flag)
	{
		if (path.empty())
		{
			for (GetSetDictionary::iterator it=dict.begin();it!=dict.end();++it)
				declare(it->first);
			return *this;
		}
		std::vector<std::string> fs=stringToVector<std::string>(dict[path]["CommandLineFlag"],';');
		for (int i=0;i<(int)fs.size();i++)
			flags[fs[i]]=path;
		if (!flag.empty())
			flags[flag]=path;
		else if (fs.empty())
		{
			std::string section=path;
			std::string name=splitRight(section,"/\\");
			std::transform(name.begin(),name.end(),name.begin(),tolower);
			std::replace(name.begin(),name.end(),' ','-');
			std::replace(name.begin(),name.end(),'/','-');
			flags[std::string("--")+name]=path;
		}
		return *this;
	}

	bool CmdLineParser::parse(int argc, char **argv)
	{
  		int unnamed=0;
		for (int i=0;i<argc;i++)
		{
			std::string arg=argv[i];
			if (arg[0]=='-' && i<=argc-2)
			{
				MapStrStr::iterator it=flags.find(arg);
				if (it!=flags.end() && dict.find(it->second)!=dict.end())
				{
					dict[it->second]["Value"]=argv[++i];
					required.erase(it->second);
					if (found.find(it->second)!=found.end())
						std::cerr << "Warning: Multiple definitions of parameters \"" << it->second << "\" via command line!\n";
					found.insert(it->second);
					continue;
				}
			}
			MapStrStr::iterator it=flags.find(toString(unnamed++));
			if (it!=flags.end() && i<argc)
			{
				dict[it->second]["Value"]=argv[i];
				required.erase(it->second);
				if (found.find(it->second)!=found.end())
					std::cerr << "Warning: Multiple definitions of parameters \"" << it->second << "\" via command line!\n";
				found.insert(it->second);
				continue;
			}
			unhandledArgs[i]=arg;
		}
		return required.empty();
	}

} // namespace GetSetIO

#endif // __GetSetCmdLine_hxx
