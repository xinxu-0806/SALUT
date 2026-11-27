#include "salut_gearbox.h"

// Concrete TAPA tasks for the cross-SLR 4:1 gearboxes (one split/join pair per record type).
void SalutGear4SplitRP(tapa::istream<RP>& in, tapa::ostream<SalutGearBeat<RP, 4>>& out) {
  SalutGearSplit<RP, 4>(in, out);
}
void SalutGear4JoinRP(tapa::istream<SalutGearBeat<RP, 4>>& in, tapa::ostream<RP>& out) {
  SalutGearJoin<RP, 4>(in, out);
}
void SalutGear4SplitWP(tapa::istream<WP>& in, tapa::ostream<SalutGearBeat<WP, 4>>& out) {
  SalutGearSplit<WP, 4>(in, out);
}
void SalutGear4JoinWP(tapa::istream<SalutGearBeat<WP, 4>>& in, tapa::ostream<WP>& out) {
  SalutGearJoin<WP, 4>(in, out);
}
void SalutGear4SplitResponse(tapa::istream<SalutResponseEnvelope>& in,
    tapa::ostream<SalutGearBeat<SalutResponseEnvelope, 4>>& out) {
  SalutGearSplit<SalutResponseEnvelope, 4>(in, out);
}
void SalutGear4JoinResponse(tapa::istream<SalutGearBeat<SalutResponseEnvelope, 4>>& in,
    tapa::ostream<SalutResponseEnvelope>& out) {
  SalutGearJoin<SalutResponseEnvelope, 4>(in, out);
}
void SalutGear4SplitWordRead(tapa::istream<SalutWordReadRsp>& in,
    tapa::ostream<SalutGearBeat<SalutWordReadRsp, 4>>& out) {
  SalutGearSplit<SalutWordReadRsp, 4>(in, out);
}
void SalutGear4JoinWordRead(tapa::istream<SalutGearBeat<SalutWordReadRsp, 4>>& in,
    tapa::ostream<SalutWordReadRsp>& out) {
  SalutGearJoin<SalutWordReadRsp, 4>(in, out);
}
void SalutGear4SplitWriteback(tapa::istream<SalutWritebackWord>& in,
    tapa::ostream<SalutGearBeat<SalutWritebackWord, 4>>& out) {
  SalutGearSplit<SalutWritebackWord, 4>(in, out);
}
void SalutGear4JoinWriteback(tapa::istream<SalutGearBeat<SalutWritebackWord, 4>>& in,
    tapa::ostream<SalutWritebackWord>& out) {
  SalutGearJoin<SalutWritebackWord, 4>(in, out);
}
void SalutGear4SplitWriteBeat(tapa::istream<SalutWriteBeat>& in,
    tapa::ostream<SalutGearBeat<SalutWriteBeat, 4>>& out) {
  SalutGearSplit<SalutWriteBeat, 4>(in, out);
}
void SalutGear4JoinWriteBeat(tapa::istream<SalutGearBeat<SalutWriteBeat, 4>>& in,
    tapa::ostream<SalutWriteBeat>& out) {
  SalutGearJoin<SalutWriteBeat, 4>(in, out);
}
