#ifndef LOCATION_HPP
#define LOCATION_HPP
enum class Location {
  VOID = -1,
  IN = 0,
  V1 = 1,
  V2 = 2,
  V3 = 3,
  V1V2 = 4,
  V1V3 = 5,
  V2V3 = 6
};
const inline char* location_to_string(Location l) {
  switch (l) {
    case Location::VOID:
      return "VOID";
    case Location::IN:
      return "IN";
    case Location::V1:
      return "V1";
    case Location::V2:
      return "V2";
    case Location::V3:
      return "V3";
    case Location::V1V2:
      return "V1V2";
    case Location::V1V3:
      return "V1V3";
    case Location::V2V3:
      return "V2V3";
    default:
      return "INVALID";
  }
};
inline Location int_to_location(int i) {
  switch (i) {
    case -1:
      return Location::VOID;
    case 0:
      return Location::IN;
    case 1:
      return Location::V1;
    case 2:
      return Location::V2;
    case 3:
      return Location::V3;
    case 4:
      return Location::V1V2;
    case 5:
      return Location::V1V3;
    case 6:
      return Location::V2V3;
    default:
      return Location::VOID;
  }
};
#endif  // LOCATION_HPP