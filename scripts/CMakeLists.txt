# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-hrprtc-hrp2
IF(GRX_PREFIX)
  FOREACH(ROBOT ${GRX_ROBOTS})
    IF (${ROBOT} STREQUAL HRP2LAAS)
      CONFIGURE_FILE (grx/sotinfo-14.py.in grx/${ROBOT}/sotinfo.py)
      CONFIGURE_FILE (grx/sot-hrp2.py.in ${CMAKE_CURRENT_BINARY_DIR}/grx/${ROBOT}/sot.py)
    ELSEIF (${ROBOT} STREQUAL HRP2JRL)
      CONFIGURE_FILE (grx/sotinfo-10.py.in grx/${ROBOT}/sotinfo.py)
      CONFIGURE_FILE (grx/sot-hrp2.py.in ${CMAKE_CURRENT_BINARY_DIR}/grx/${ROBOT}/sot.py)
    ELSEIF (${ROBOT} STREQUAL HRP4LIRMM)
      CONFIGURE_FILE (grx/sotinfo-hrp4.py.in grx/${ROBOT}/sotinfo.py)
      CONFIGURE_FILE (grx/sot-hrp4.py.in ${CMAKE_CURRENT_BINARY_DIR}/grx/${ROBOT}/sot.py)
    ENDIF ()

    SET(SCRIPT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/grx/guisot.py
      ${CMAKE_CURRENT_SOURCE_DIR}/grx/guiinfosot.py
      ${CMAKE_CURRENT_BINARY_DIR}/grx/${ROBOT}/sot.py
      ${CMAKE_CURRENT_BINARY_DIR}/grx/${ROBOT}/sotinfo.py)
    INSTALL(FILES ${SCRIPT_FILES}
      DESTINATION ${GRX_PREFIX}/${ROBOT}/script
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ GROUP_WRITE WORLD_READ WORLD_EXECUTE)
  ENDFOREACH(ROBOT)
ENDIF(GRX_PREFIX)
