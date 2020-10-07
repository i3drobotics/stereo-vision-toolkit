
# INCLUDEPATH += $$PWD
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
)

set(QTAWESOME_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# SOURCES += $$PWD/QtAwesome.cpp \  
    # $$PWD/QtAwesomeAnim.cpp
set(SOURCES
	${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/QtAwesome.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/QtAwesomeAnim.cpp
	PARENT_SCOPE
)

# HEADERS += $$PWD/QtAwesome.h \   
    # $$PWD/QtAwesomeAnim.h
set(MOC_HEADERS
	${MOC_HEADERS}
	${CMAKE_CURRENT_SOURCE_DIR}/QtAwesome.h
	${CMAKE_CURRENT_SOURCE_DIR}/QtAwesomeAnim.h
	PARENT_SCOPE
)
    
# RESOURCES += $$PWD/QtAwesome.qrc
set(RESOURCES
	${RESOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/QtAwesome.qrc
	PARENT_SCOPE
)