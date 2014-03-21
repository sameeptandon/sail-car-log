#if defined _MSC_VER || defined __GNUC__
  #pragma pack(pop)
#endif

#if defined _MSC_VER
  //Disable warning C4103: alignment changed after including header, may be due to missing #pragma pack(pop)
  #pragma warning(disable:4103)

  //Default warning C4201: nonstandard extension used : nameless struct/union
  #pragma warning(default:4201)
#endif
