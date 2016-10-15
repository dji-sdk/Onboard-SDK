set(EXTERNAL_ROOT_DIR /usr/include /usr/local/include /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

# Find PCAP headers
find_path(PCAP_INCLUDE_DIR
  NAMES pcap/pcap.h
  HINTS ${EXTERNAL_ROOT_DIR}
  NO_DEFAULT_PATH)

find_library(PCAP_LIBRARY
  NAMES pcap
  HINTS ${EXTERNAL_ROOT_DIR}
  NO_DEFAULT_PATH)

# Make these available for the user to set
mark_as_advanced(CLEAR PCAP_INCLUDE_DIR)
mark_as_advanced(CLEAR PCAP_LIBRARY)

if(PCAP_INCLUDE_DIR)
  message(STATUS " Pcap include dirs set to ${PCAP_INCLUDE_DIR}")
  set(PCAP_FOUND "YES")
endif()

if(PCAP_LIBRARY)
  message(STATUS " Pcap lib set to ${PCAP_LIBRARY}")
endif()
