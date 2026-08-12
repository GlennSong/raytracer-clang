#ifndef ROADLAB_RL_XML_H
#define ROADLAB_RL_XML_H

// A minimal XML reader, sized for OpenDRIVE and nothing more.
//
// The project's accepted dependency set has a JSON library but no XML one, and
// .xodr uses a small, well-behaved subset: elements, attributes, self-closing
// tags, comments, a processing instruction, and the five predefined entities.
// No namespaces, no CDATA, no DTDs, no mixed content that matters. Parsing that
// is a couple of hundred lines, which is cheaper than adding a dependency to a
// prototype that is meant to stay buildable with the STL alone.
//
// It is strict about structure (mismatched tags are an error with a line number)
// and lenient about everything it does not need.

#include <string>
#include <vector>

namespace roadlab {

struct XmlNode {
    std::string name;
    std::vector<std::pair<std::string, std::string>> attrs;
    std::vector<XmlNode> children;

    const std::string* attr(const char* key) const;
    bool has(const char* key) const { return attr(key) != nullptr; }
    double attrD(const char* key, double fallback = 0.0) const;
    int attrI(const char* key, int fallback = 0) const;
    std::string attrS(const char* key, const char* fallback = "") const;

    const XmlNode* child(const char* childName) const;
    std::vector<const XmlNode*> all(const char* childName) const;
};

// Parses `text` into `root` (the document's single top-level element). Returns
// false with a message on malformed input.
bool xmlParse(const std::string& text, XmlNode& root, std::string& error);
bool xmlParseFile(const std::string& path, XmlNode& root, std::string& error);

}  // namespace roadlab

#endif
