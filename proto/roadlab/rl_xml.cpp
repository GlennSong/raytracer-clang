#include "rl_xml.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace roadlab {

const std::string* XmlNode::attr(const char* key) const {
    for (const auto& kv : attrs)
        if (kv.first == key) return &kv.second;
    return nullptr;
}

double XmlNode::attrD(const char* key, double fallback) const {
    const std::string* v = attr(key);
    return v ? std::atof(v->c_str()) : fallback;
}

int XmlNode::attrI(const char* key, int fallback) const {
    const std::string* v = attr(key);
    return v ? std::atoi(v->c_str()) : fallback;
}

std::string XmlNode::attrS(const char* key, const char* fallback) const {
    const std::string* v = attr(key);
    return v ? *v : std::string(fallback);
}

const XmlNode* XmlNode::child(const char* childName) const {
    for (const XmlNode& c : children)
        if (c.name == childName) return &c;
    return nullptr;
}

std::vector<const XmlNode*> XmlNode::all(const char* childName) const {
    std::vector<const XmlNode*> out;
    for (const XmlNode& c : children)
        if (c.name == childName) out.push_back(&c);
    return out;
}

namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;
    int line = 1;
    std::string error;

    explicit Parser(const std::string& text) : s(text) {}

    bool eof() const { return i >= s.size(); }
    char peek() const { return i < s.size() ? s[i] : '\0'; }
    char get() {
        char c = s[i++];
        if (c == '\n') ++line;
        return c;
    }
    bool starts(const char* lit) const { return s.compare(i, std::strlen(lit), lit) == 0; }

    void skipSpace() {
        while (!eof() && (peek() == ' ' || peek() == '\t' || peek() == '\r' || peek() == '\n'))
            get();
    }

    bool fail(const std::string& what) {
        if (error.empty()) error = "line " + std::to_string(line) + ": " + what;
        return false;
    }

    // Comments, processing instructions and doctypes are skipped wholesale; none
    // of them carry anything an .xodr reader needs.
    void skipTrivia() {
        for (;;) {
            skipSpace();
            if (starts("<!--")) {
                size_t end = s.find("-->", i);
                if (end == std::string::npos) {
                    i = s.size();
                    return;
                }
                for (; i < end + 3; ) get();
            } else if (starts("<?")) {
                size_t end = s.find("?>", i);
                if (end == std::string::npos) {
                    i = s.size();
                    return;
                }
                for (; i < end + 2; ) get();
            } else if (starts("<!")) {
                size_t end = s.find('>', i);
                if (end == std::string::npos) {
                    i = s.size();
                    return;
                }
                for (; i <= end; ) get();
            } else {
                return;
            }
        }
    }

    static void unescape(std::string& v) {
        static const std::pair<const char*, char> kEntities[] = {
            {"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}};
        for (const auto& e : kEntities) {
            size_t at = 0;
            size_t n = std::strlen(e.first);
            while ((at = v.find(e.first, at)) != std::string::npos) {
                v.replace(at, n, 1, e.second);
                at += 1;
            }
        }
    }

    bool readName(std::string& out) {
        out.clear();
        while (!eof()) {
            char c = peek();
            if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == ':' || c == '-' ||
                c == '.') {
                out += get();
            } else {
                break;
            }
        }
        return !out.empty();
    }

    bool readElement(XmlNode& node) {
        if (peek() != '<') return fail("expected '<'");
        get();
        if (!readName(node.name)) return fail("expected an element name");

        for (;;) {
            skipSpace();
            if (eof()) return fail("unterminated element <" + node.name + ">");
            if (peek() == '/') {
                get();
                if (peek() != '>') return fail("expected '>' after '/'");
                get();
                return true;   // self-closing
            }
            if (peek() == '>') {
                get();
                break;
            }
            std::string key;
            if (!readName(key)) return fail("expected an attribute name");
            skipSpace();
            if (peek() != '=') return fail("expected '=' after attribute " + key);
            get();
            skipSpace();
            char quote = peek();
            if (quote != '"' && quote != '\'') return fail("expected a quoted attribute value");
            get();
            std::string value;
            while (!eof() && peek() != quote) value += get();
            if (eof()) return fail("unterminated attribute value");
            get();
            unescape(value);
            node.attrs.push_back({key, value});
        }

        // Children, until the matching close tag. Text content is discarded:
        // OpenDRIVE keeps its data in attributes.
        for (;;) {
            skipTrivia();
            if (eof()) return fail("unterminated element <" + node.name + ">");
            if (peek() != '<') {
                get();   // stray text
                continue;
            }
            if (starts("</")) {
                get();
                get();
                std::string closing;
                readName(closing);
                skipSpace();
                if (peek() != '>') return fail("expected '>' in closing tag");
                get();
                if (closing != node.name)
                    return fail("closing </" + closing + "> does not match <" + node.name + ">");
                return true;
            }
            node.children.emplace_back();
            if (!readElement(node.children.back())) return false;
        }
    }
};

}  // namespace

bool xmlParse(const std::string& text, XmlNode& root, std::string& error) {
    Parser p(text);
    p.skipTrivia();
    if (p.eof()) {
        error = "empty document";
        return false;
    }
    root = XmlNode{};
    if (!p.readElement(root)) {
        error = p.error;
        return false;
    }
    return true;
}

bool xmlParseFile(const std::string& path, XmlNode& root, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return xmlParse(ss.str(), root, error);
}

}  // namespace roadlab
