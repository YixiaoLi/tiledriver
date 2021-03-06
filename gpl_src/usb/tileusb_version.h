#define TILEUSB_MAJOR_VER 1
#define TILEUSB_MINOR_VER 0
#define TILEUSB_FEATURES 0x0

#define STRIFY(X) #X
#define MAKE_VERSION_STRING(MAJOR, MINOR, FEATURES) \
	STRIFY(MAJOR) "." STRIFY(MINOR) "-" STRIFY(FEATURES) "-4.2.3.172691"

#define TILEUSB_VERSION_STRING \
	MAKE_VERSION_STRING(TILEUSB_MAJOR_VER, TILEUSB_MINOR_VER, \
			    TILEUSB_FEATURES)

#define LICENSE_STRING "GPL"
