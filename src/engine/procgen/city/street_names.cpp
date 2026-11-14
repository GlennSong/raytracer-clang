#include "street_names.h"
#include "../../../proc_rng.h"

#include "../../asset_root.h"
#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_set>

namespace engine {

namespace {

// The FALLBACK book: assets/data/streets.json is the content (see the header).
// This is only what a stripped install falls back to, so a city still has
// street names when the asset is missing.
StreetNameBook fallbackBook() {
    StreetNameBook b;
    b.banks = {{"Oak", "Maple", "Elm", "Cedar", "Willow", "Birch", "Aspen", "Alder"},
               {"Lincoln", "Jefferson", "Franklin", "Madison", "Monroe", "Jackson"},
               {"Market", "Mill", "Depot", "Union", "Commerce", "Exchange"},
               {"First", "Second", "Third", "Fourth", "Fifth", "Sixth"}};
    b.suffixes = {{16, {"Boulevard", "Avenue", "Parkway"}},
                  {13, {"Avenue", "Road", "Way"}},
                  {9, {"Street", "Road", "Drive"}},
                  {0, {"Lane", "Court", "Place"}}};
    b.classSuffix = {{"alley", "Alley"}};
    b.abbreviations = {{"Boulevard", "Blvd"}, {"Parkway", "Pkwy"}, {"Avenue", "Ave"},
                       {"Street", "St"}, {"Road", "Rd"}, {"Drive", "Dr"}, {"Lane", "Ln"},
                       {"Court", "Ct"}, {"Place", "Pl"}, {"Alley", "Aly"}};
    b.unnamed = {"freeway", "ramp"};
    return b;
}

// The suffix for a run: its CLASS if the book names one (an alley is an
// Alley whatever its width), else the widest rule it meets.
std::string suffixFor(const StreetNameBook& b, double width, const std::string& roadClass, ProcRng& r) {
    const auto byClass = b.classSuffix.find(roadClass);
    if (byClass != b.classSuffix.end()) return byClass->second;
    for (const StreetNameBook::SuffixRule& rule : b.suffixes)
        if (width >= rule.minWidth && !rule.choices.empty())
            return rule.choices[r.below(static_cast<int>(rule.choices.size()))];
    return "Street";
}

}  // namespace

StreetNameBook parseStreetNameBook(const nlohmann::json& names) {
    StreetNameBook b;
    if (!names.is_object()) return b;
    b.seed = names.value("seed", 20260918u);
    if (names.contains("banks") && names["banks"].is_object())
        for (const auto& [tag, words] : names["banks"].items()) {
            std::vector<std::string> pool;
            for (const auto& w : words)
                if (w.is_string()) pool.push_back(w.get<std::string>());
            if (!pool.empty()) b.banks.push_back(std::move(pool));
        }
    if (names.contains("suffixes"))
        for (const auto& rule : names["suffixes"]) {
            StreetNameBook::SuffixRule sr;
            sr.minWidth = rule.value("minWidth", 0.0);
            for (const auto& c : rule.value("choices", nlohmann::json::array()))
                if (c.is_string()) sr.choices.push_back(c.get<std::string>());
            if (!sr.choices.empty()) b.suffixes.push_back(std::move(sr));
        }
    std::stable_sort(b.suffixes.begin(), b.suffixes.end(),
                     [](const StreetNameBook::SuffixRule& x, const StreetNameBook::SuffixRule& y) {
                         return x.minWidth > y.minWidth;   // widest rule first
                     });
    if (names.contains("classSuffix") && names["classSuffix"].is_object())
        for (const auto& [k, v] : names["classSuffix"].items())
            if (v.is_string()) b.classSuffix[k] = v.get<std::string>();
    if (names.contains("abbreviations") && names["abbreviations"].is_object())
        for (const auto& [k, v] : names["abbreviations"].items())
            if (v.is_string()) b.abbreviations[k] = v.get<std::string>();
    if (names.contains("unnamed") && names["unnamed"].is_array())
        for (const nlohmann::json& c : names["unnamed"])
            if (c.is_string()) b.unnamed.insert(c.get<std::string>());
    return b;
}

const StreetNameBook& streetNameBook() {
    static StreetNameBook book = [] {
        StreetNameBook b;
        const std::string path = assetPath("assets/data/streets.json");
        std::ifstream in(path);
        if (in) {
            nlohmann::json doc;
            try {
                in >> doc;
                b = parseStreetNameBook(doc.value("names", nlohmann::json::object()));
                b.fromAsset = b.usable();
            } catch (const std::exception& e) {
                LOG_WARN << "[streets] " << path << ": " << e.what();
            }
        }
        if (!b.usable()) {
            LOG_WARN << "[streets] no usable assets/data/streets.json: "
                        "falling back to the built-in name book";
            b = fallbackBook();
        }
        return b;
    }();
    return book;
}

std::vector<std::string> nameRuns(const std::vector<RunToName>& runs, const StreetNameBook& book, uint32_t seed,
                                  const std::set<std::string>& taken) {
    std::vector<std::string> out(runs.size());
    // Longest runs name first so the main roads draw from the full bank.
    std::vector<int> order(runs.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return runs[static_cast<std::size_t>(a)].length > runs[static_cast<std::size_t>(b)].length;
    });
    ProcRng rng(seed ? seed : book.seed);
    // The WORD is unique while the bank lasts: "Hamilton Rd" and "Hamilton Dr"
    // in one city is a wrong turn waiting to happen. Past that, the full name.
    std::unordered_set<std::string> names(taken.begin(), taken.end()), words;
    for (const std::string& t : taken) words.insert(t.substr(0, t.find(' ')));
    const std::size_t bankCount = book.banks.size();
    for (int ri : order) {
        const RunToName& run = runs[static_cast<std::size_t>(ri)];
        std::string name;
        for (int attempt = 0; attempt < 96 && bankCount; ++attempt) {
            const std::vector<std::string>& bank = book.banks[rng.below(static_cast<int>(bankCount))];
            const std::string word = bank[rng.below(static_cast<int>(bank.size()))];
            name = word + " " + suffixFor(book, run.width, run.roadClass, rng);
            const bool wordFree = !words.count(word);
            if (!names.count(name) && (wordFree || attempt >= 64)) {
                words.insert(word);
                break;
            }
            name.clear();
        }
        if (name.empty() && bankCount) {   // the banks ran dry: number it
            const std::vector<std::string>& bank = book.banks[rng.below(static_cast<int>(bankCount))];
            name = bank[rng.below(static_cast<int>(bank.size()))] + " " + suffixFor(book, run.width, run.roadClass, rng) +
                   " " + std::to_string(ri);
        }
        names.insert(name);
        out[static_cast<std::size_t>(ri)] = std::move(name);
    }
    return out;
}

std::string abbreviateStreetName(const std::string& name) {
    return abbreviateStreetName(name, streetNameBook());
}

std::string abbreviateStreetName(const std::string& name, const StreetNameBook& book) {
    for (const auto& [full, abbr] : book.abbreviations) {
        const std::string suffix = std::string(" ") + full;
        const std::size_t at = name.rfind(suffix);
        if (at != std::string::npos &&
            (at + suffix.size() == name.size() || name[at + suffix.size()] == ' '))
            return name.substr(0, at) + " " + abbr + name.substr(at + suffix.size());
    }
    return name;
}

}  // namespace engine
