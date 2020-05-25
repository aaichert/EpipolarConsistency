//
// Header-only version of basic GetSet functionality
//  by André Aichert (aaichert@gmail.com)
//
//
// Rights to modify and/or re-distribute this file are explicitly granted.
// I, the creator of this file, do not claim any rights.
//
// Copy this file to your project to use.
//

#include "StringType.hxx"
#include "StringUtil.hxx"
#include "StringConversion.hxx"

#include <iostream>

#ifndef __getset_minimal_hxx
#define __getset_minimal_hxx

#include <set>

// Return n-th element in an semicolon seperated list
inline std::string enumNth(int n, const std::string& choices)
{
	std::vector<std::string> c=stringToVector<std::string>(choices,';');
	if (n>=0 && n<(int)c.size())
		return c[n];
	else
		return "";
}

// Find item in a semicolon seperated list. Returns -1 if not found
inline int enumIdx(const std::string& item, const std::string& choices)
{
	std::vector<std::string> c=stringToVector<std::string>(choices,';');
	for (int i=0;i<(int)c.size();i++)
		if (c[i]==item) return i;
	return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GetSet class

// Define the static instance of the GetSet dictionary
#define GETSET_GLOBAL_DICTIONARY GetSetDictionary* GetSetDictionary::_instance=0x0;

typedef std::map<std::string, std::string> MapStrStr;
typedef std::map<std::string, std::map<std::string, std::string> > MapStrMapStrStr;

// This class privately holds all properties.
class GetSetDictionary : public MapStrMapStrStr
{
public:
	/// Access to the global GetSetDictionary, which is used whenever no Dictionary is explicitly specified.
	static GetSetDictionary& global()
	{
		if (!_instance)
		_instance=new GetSetDictionary();
		return *_instance;
	}
protected:
	/// The instance that holds the global() dictionary. Used whenever a GetSetDictionary is not explicitly specified.
	static GetSetDictionary *_instance;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Syntactic sugar to access and change GetSet properties
template <typename BasicType=std::string>
class GetSet
{
public:
	/// Access a GetSet property by the absolute path to its key (and optionally explicitly a dictionary)
	GetSet(const std::string& path, GetSetDictionary& _dict=GetSetDictionary::global())
		: key(path) , dict(_dict)
	{
		if (dict[key]["Type"]=="string" || dict[key]["Type"].empty())
			dict[key]["Type"]=typeName<BasicType>();
	}

	/// Set the value of a GetSet property (same as: assigment operator)
	inline GetSet<BasicType>& setValue(const BasicType& v)
	{
		if (dict[key]["Type"]=="Enum" && typeName<BasicType>()!="string") 
			dict[key]["Value"]=enumNth(stringTo<int>(toString(v)),getAttribute("Choices"));
		else setString(toString(v));
		return *this;
	}
	/// Get the value of a GetSet property (same as: cast operator)
	inline const BasicType getValue() const
	{
		if (dict[key]["Type"]=="Enum" && typeName<BasicType>()!="string") 
			return stringTo<BasicType>(toString(enumIdx(dict[key]["Value"],getAttribute("Choices"))));
		return stringTo<BasicType>(dict[key]["Value"]);
	}

	/// Set the value of this property from a string
	inline GetSet<BasicType>& setString(const std::string& value) { dict[key]["Value"]=value; return *this; }
	/// Get the value of the property as string
	inline std::string getString() const { return dict[key]["Value"]; }

	/// Set the value of a GetSet property directly via assignment operator
	inline GetSet<BasicType>& operator=(const BasicType& v) { setValue(v); return *this;}
	/// Cast operator directly to BasicType (behaves almost like a c++ variable of BasicType)
	inline operator BasicType() const { return getValue(); }

	/// Set a brief description for this property. Same as setAttribute("Description",...).
	inline GetSet<BasicType>& setDescription(const std::string& desc) {setAttribute("Description",desc); return *this;}
	/// Get a brief description for this property. Same as getAttribute("Description").
	inline std::string getDescription() const {return getAttribute("Description");}

	/// Access attributes directly (eg. "Description", "CommandLineArg" etc.)
	inline GetSet<BasicType>& setAttribute(const std::string& attrib, const std::string& value) {dict[key][attrib]=value;return*this;}
	/// Access attributes directly
	inline std::string getAttribute(const std::string& attrib) const  {
		return dict[key][attrib];
	}

protected:
	// Path to key
	std::string key;
	// The properties
	// mutable
    GetSetDictionary& dict;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Special GetSet types:
namespace GetSetGui {
	#define GETSET_SPECIALIZATION(SPECIAL_TYPE,BASE_TYPE,CLASS_BODY, IGNORED)							\
		class SPECIAL_TYPE : public GetSet<BASE_TYPE> {													\
		public:																							\
			SPECIAL_TYPE(const std::string& key, GetSetDictionary& dict=GetSetDictionary::global())		\
				: GetSet<BASE_TYPE>(key, dict) {														\
				setAttribute("Type", #SPECIAL_TYPE );													\
			}																							\
			SPECIAL_TYPE& operator=(const BASE_TYPE& v) { setValue(v); return *this; }					\
			operator BASE_TYPE() const { return getValue(); }											\
			CLASS_BODY																					\
		};

	#define GETSET_TAG(SPECIAL_TYPE,TYPE,TAG)															\
		SPECIAL_TYPE& set##TAG(const TYPE& value) {setAttribute(#TAG,toString(value));return *this;}	\
		TYPE get##TAG() const {return stringTo<TYPE>(getAttribute(#TAG));}

	// The Enum class is more complex, because it has features of both GetSet<std::string> and GetSet<int>
	#define GETSET_ENUM_CLASS_BODY																		\
		GETSET_TAG(Enum,std::vector<std::string>,Choices)												\
		Enum& setChoices(const std::string& c) {setAttribute("Choices",c);return *this;}				\
		inline void operator=(const std::string& v) {setString(v);}										\
		inline operator std::string() const {return getString();}
	
	/// A pulldown menu with a number of choices.
	GETSET_SPECIALIZATION(Enum,int,GETSET_ENUM_CLASS_BODY, )

	/// A GetSet&lt;double&gt; with additional range information, so that it could be represented as a slider
	GETSET_SPECIALIZATION(Slider,double, GETSET_TAG(Slider,double,Min) GETSET_TAG(Slider,double,Max), )

	/// A button that creates a GetSet change event when pressed. Can be tagged with additional info for script execution.
	GETSET_SPECIALIZATION(Button,std::string, GETSET_TAG(Button,std::string,Script) void trigger() { setString(getString()); }, )

	/// A static text with some information. StaticTexts are not included in ini-Files (user-info in GUI)
	GETSET_SPECIALIZATION(StaticText,std::string, , )

	/// An edit field, but read-only. Intended for output-values that the user can select and copy to clipboard.
	GETSET_SPECIALIZATION(ReadOnlyText,std::string, , )

	/// A directory
	GETSET_SPECIALIZATION(Directory,std::string, , )

	/// A file (or multiple semicolon seperated files). Extensions is a string such as "Images (*.png *.xpm *.jpg);;All files (*)"
	GETSET_SPECIALIZATION(File,std::string, GETSET_TAG(File,std::string,Extensions) GETSET_TAG(File,bool, CreateNew) GETSET_TAG(File,bool, Multiple), )
} // namespace GetSetGUI

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AutoGUI specific
namespace GetSetMinimal
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Parsing the contents of an ini-File
	inline void parseIni(const std::string& ini, MapStrMapStrStr& contents=GetSetDictionary::global())
	{
		std::istringstream istr(ini);
		std::string section,key,value;
		for (int lineNumber=0; !istr.eof() && istr.good(); lineNumber++)
		{
			std::string line;
			getline(istr,line,'\n');
			if (line.length()<2||line[0]=='#') continue;
			if (line[0]=='[') {
				section=line.substr(1,line.length()-2);
				continue;
			}
			std::istringstream linestr(line);
			getline(linestr,key,'=');
			getline(linestr,value,'\0');
			trim(key);
			trim(value);
			std::string path=section+(section.empty()?"":"/")+key;
			contents[path]["Value"]=value;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define ini reading capability
	inline bool loadIni(const std::string& path, MapStrMapStrStr& contents=GetSetDictionary::global())
	{
		std::ifstream istr(path.c_str());
		if (!istr.good()) return false;
		std::string all;
		std::getline(istr,all,'\0');
		parseIni(all,contents);		
		return true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Compose "[section] key=value"-style contents of an ini-File
	inline std::string getIni(MapStrMapStrStr& contents=GetSetDictionary::global())
	{
		std::ostringstream ostr;
		MapStrStr helper;
		std::set<std::string> ignore_types;
		ignore_types.insert("Button");
		ignore_types.insert("StaticText");
		for (MapStrMapStrStr::const_iterator sectit=contents.begin();sectit!=contents.end();++sectit)
		{
			MapStrStr::const_iterator t=sectit->second.find("Type");
			if (t!=sectit->second.end() && ignore_types.find(t->second)!=ignore_types.end())
				continue;
			std::string section=sectit->first;
			std::string key=splitRight(section,"/\\");
			MapStrStr::const_iterator value=sectit->second.find("Value");
			if (value!=sectit->second.end())
				helper[std::string("[")+section+"]\n"] += key + " = " + value->second + "\n";
		}
		for (MapStrStr::iterator it=helper.begin();it!=helper.end();++it)
			ostr << std::endl << it ->first << it->second << std::endl;
		return ostr.str();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define ini writing capability
	inline bool saveIni(const std::string& path, MapStrMapStrStr& contents=GetSetDictionary::global())
	{
		std::ofstream ostr(path.c_str());
		if (!ostr.good()) return false;
		ostr << getIni(contents);
		return true;
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define Function to set progress bar
	inline void progress(const std::string& identifier, int i, int n) {
			std::cout << "### Progress - " << identifier << " : status - " << i << " / " << n << std::endl;
	}
	
	inline void progress_hide(const std::string& identifier) {
			std::cout << "### Progress - " << identifier << " : hide - " << std::endl;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define Function set display text
	inline void info(const std::string& identifier, const std::string text)
	{
		std::cout << "### Info - " << identifier << " : - " << text << std::endl;
	}
	inline void info_hide(const std::string& identifier)
	{
		std::cout << "### Info - " << identifier << " : hide - " << std::endl;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define Function for warnings (message box)
	inline void warning(const std::string& identifier, const std::string& text)
	{
		std::cout << "### Message - " << identifier << " : warning - " << text << std::endl;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define Function to abort abnormally with an error message (message box)
	inline void error(const std::string& identifier, const std::string& text, int err=1)
	{
		std::cout << "### Message - " << identifier << " : error - " << text << std::endl;
		exit(err);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define Function to hide window
	inline void hide(const std::string& identifier)
	{
		std::cout << "Window - " << identifier << " : hide -\n";
	}

} // namespace GetSetAutoGUI

#endif // __getset_minimal_hxx
