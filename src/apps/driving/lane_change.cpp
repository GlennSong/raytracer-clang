#include "apps/driving/lane_change.h"

namespace driving {

namespace {

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

FactSpec spec(const char* name, FactKind kind, UnitFamily unit, NoticeFn fn, const char* meaning) {
    FactSpec s;
    s.name = name;
    s.kind = kind;
    s.unit = unit;
    s.notice = fn;
    s.meaning = meaning;
    return s;
}

void noticeWrongLane(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange) return;
    out.present = true;
    out.yes = v.laneChange->need.onRoute && v.laneChange->need.lanes > 0;
}

void noticeLastChance(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange || !v.laneChange->last.needed) return;
    out.present = true;
    out.number = v.laneChange->last.metres;
}

void noticeGapBeside(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange) return;
    out.present = true;
    out.yes = v.laneChange->gap.safe;
    if (v.laneChange->gap.target.valid()) out.aboutPath = v.laneChange->gap.target;
}

void noticeMustWaitToChange(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange) return;
    out.present = true;
    out.yes = v.laneChange->mustWait && v.laneChange->need.lanes > 0;
}

void noticeChangeWaitLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange || !v.laneChange->mustWait || v.laneChange->need.lanes == 0) return;
    const Real metres = std::max(Real(0), v.laneChange->changeBy);
    out.present = true;
    out.number = metres;
    out.hasMetres = true;
    out.metres = metres;
    out.aboutPath = v.me.path;
}

void noticeCarWantingIn(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.laneChange) return;
    out.present = true;
    out.yes = v.laneChange->wantingIn.valid();
}

}  // namespace

bool registerLaneChangeFacts(FactRegistry& registry, std::vector<std::string>* problems) {
    std::vector<FactSpec> specs;
    specs.push_back(spec(kFactWrongLane, FactKind::YesNo, UnitFamily::Ratio, noticeWrongLane,
                         "my route needs another lane of this road: a lane beside mine leads on where mine does not"));
    specs.push_back(spec(kFactLastChance, FactKind::Number, UnitFamily::Length, noticeLastChance,
                         "how far I may still stay in my lane before my route can no longer be kept from it (m)"));
    specs.push_back(spec(kFactMustWaitToChange, FactKind::YesNo, UnitFamily::Ratio, noticeMustWaitToChange,
                         "nothing leads on from my lane to where I go: I must get into the lane beside, and wait for a gap"));
    specs.push_back(spec(kFactChangeWaitLine, FactKind::Place, UnitFamily::Length, noticeChangeWaitLine,
                         "where I wait for that gap: the last place a move across still fits, from my front (m)"));
    specs.push_back(spec(kFactCarWantingIn, FactKind::YesNo, UnitFamily::Ratio, noticeCarWantingIn,
                         "a car in the lane beside, just ahead of me, is indicating into my lane (and I let no other in "
                         "that is not across yet)"));
    FactSpec gap = spec(kFactGapBeside, FactKind::YesNo, UnitFamily::Ratio, noticeGapBeside,
                        "there is a safe gap in the lane beside me toward the one my route needs: after moving in, the car "
                        "behind me there and I each brake no harder than lane_change_safe_brake (MOBIL's safety criterion)");
    gap.numbers.push_back(need(kNumberLaneChangeBrake, UnitFamily::Acceleration));
    specs.push_back(gap);

    bool ok = true;
    for (FactSpec& s : specs) {
        std::string why;
        if (registry.add(std::move(s), &why) == kNoFact) {
            ok = false;
            if (problems) problems->push_back(why);
        }
    }
    ReaderSpec reader;
    reader.name = kLaneChangeReaderName;
    reader.meaning = "looks for a gap in the lane beside for each driver whose route needs it, and carries out the move";
    reader.numbers.push_back(need(kNumberLaneChangeBrake, UnitFamily::Acceleration));
    reader.numbers.push_back(need(kNumberLaneChangeSeconds, UnitFamily::Time));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        ok = false;
        if (problems) problems->push_back(why);
    }
    return ok;
}

bool registerLaneChangeAnswers(AnswerVocabulary& vocabulary, std::string* why) {
    return vocabulary.addIntention(kIntentionLane, {kWordChange}, "move over into the lane beside, toward the one my route needs",
                                   why) != kNoIntentionKind &&
           vocabulary.addIntention(kIntentionLetIn, {kWordLetIn},
                                   "ease off to let the car indicating into my lane ahead of me in: keep behind where it would be",
                                   why) != kNoIntentionKind;
}

bool hasLaneChangeStep(const FactRegistry& registry) { return registry.findReader(kLaneChangeReaderName) != nullptr; }

}  // namespace driving
