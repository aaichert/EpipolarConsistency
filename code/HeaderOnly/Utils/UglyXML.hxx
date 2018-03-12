#ifndef __ugly_xml_hxx
#define __ugly_xml_hxx
// An ugly and non-standard conforming XML parser
// Actually intended ONLY for GetSet Parameters.
#include <iostream>
#include "../NRRD/StringUtil.hxx"

#include <GetSet/ProgressInterface.hxx>

namespace GetSetInternal
{
	/// A rudimentary non-standard conforming XML representation/parser.
	class UglyXML {
	public:

		struct Tag {
			size_t from;
			size_t to;
			std::string tag;
			std::map<std::string,std::string> attrib;
			std::string text;
			std::vector<Tag> tags;

			// Appends to found_items all tag (optionally, only those that have a certain attribute and, also optionally, a certain value)
			inline void dfs_search_tag(const std::string& tag_name, std::vector<Tag*>& found_items, const std::string with_attrib="", const std::string with_value="" ) 
			{
				for (auto it=tags.begin(); it!=tags.end(); ++it)
				{
					if (it->tag==tag_name) {
						if (with_attrib.empty())	
							found_items.push_back(&(*it));
						else
						{
							auto a=it->attrib.find(with_attrib);
							if (a!=it->attrib.end())
							{
								if (with_value.empty() || a->second==with_value)
									found_items.push_back(&(*it));
							}
						}
					}
					it->dfs_search_tag(tag_name,found_items,with_attrib,with_value);
				}
			}

		};
		std::vector<Tag> tags;

		// Try to parse an xml (definition see below)
		inline bool parse(const std::string& value, GetSetGui::ProgressInterface* p=0x0);

		// Depth first search for certain tag, optionally, only those that have a certain tag, and, alsooptionally, with a certain value of that tag.
		inline std::vector<Tag*> dfs_search_tag(const std::string& tag_name, const std::string with_attrib="", const std::string with_value="" )
		{
			std::vector<Tag*> ret;
			for (auto it=tags.begin(); it!=tags.end(); ++it)
				it->dfs_search_tag(tag_name,ret,with_attrib,with_value);
			return ret;
		}

		UglyXML(const std::string& value, GetSetGui::ProgressInterface *p=0x0)
		{
			if (p) p->progressStart("UglyXML","Parsing XML file (slow)...",(int)value.length(),0x0);
			if (!parse(value,p))
				std::cerr << "UglyXML failed.\n";
			if (p) p->progressEnd();
		}
	};

	inline bool parse(const std::string& value);

} // GetSetInternal


//
// Implementation of XML parser:
//

namespace GetSetInternal
{

	bool UglyXML::parse(const std::string& value, GetSetGui::ProgressInterface* p)
	{
		std::vector<Tag> openTags;
		size_t start,end=0;
		for (int num_tags=0;;num_tags++)
		{
			// Find next xml tag:
			start=value.find("<",end);
			if (p && num_tags%100==0) p->progressUpdate((int)start);
			// end=value.find(">",end+1); // fails for Type="vector<int>", hence the foor loop below.
			bool escaped=0;
			for (end=start;end<value.size();end++)
			{
				if (value[end]=='"') escaped=!escaped;
				if (!escaped&&value[end]=='>')
					break;
			}
			// If no new tag could be found
			if (start==std::string::npos||end==std::string::npos||value[end]!='>')
			{
				// eof was reached, so we better not expect any more close tags
				if (openTags.empty() && start==end) return true;
				tags.clear();
				return 0;
			}
			if (value[start+1]=='?')
				continue; // Skip declarations
			if (value[start+1]=='!')
				continue; // Skip "DOCTYPE" etc.
			if (value.substr(start+1,3)=="!--")
			{
				/// Skip comments
				end = value.find("-->",start+4);
				if (end!=std::string::npos) end+=3;
				continue;
			}
			if (value[start+1]=='/')
			{
				// This is a close tag.
				std::string closedTag=value.substr(start+2,end-start-2);
				// We expect no more tags but there is another close tag
				if (openTags.empty())
				{
					tags.clear();
					return false;
				}
				// The tag MUST match the most recently opened tag.
				if (openTags.back().tag==closedTag)
				{
					openTags.back().to=start;
					if (openTags.back().tags.empty())
						openTags.back().text=value.substr(openTags.back().from,openTags.back().to-openTags.back().from);
					if (openTags.size()==1)
					{
						tags.push_back(openTags.front());
						openTags.clear();
					}
					else
					{
						openTags[openTags.size()-2].tags.push_back(openTags.back());
						openTags.pop_back();
					}
				}
				else
				{
					// Closed tag does not match opened tag
					tags.clear();
					return false;
				}
			}
			else
			{
				// Open tag: find end of tag text
				size_t endOfText=value.find_first_of(" \t>",start);
				openTags.push_back(Tag());
				openTags.back().tag=value.substr(start+1,endOfText-start-1);
				openTags.back().from=end+1;
				std::string attribstr=value.substr(endOfText,end-endOfText);
				attrib_parse(attribstr,openTags.back().attrib);
				if (!attribstr.empty() && attribstr[attribstr.length()-1]=='/')
				{
					// empty element tag <Tag arg="value"/> style
					openTags.back().to=openTags.back().from;
					if (openTags.size()==1)
					{
						tags.push_back(openTags.front());
						openTags.clear();
					}
					else
					{
						openTags[openTags.size()-2].tags.push_back(openTags.back());
						openTags.pop_back();
					}
				}
			}
		}
	}

} // namespace GetSetInternal

#endif // __ugly_xml_hxx
