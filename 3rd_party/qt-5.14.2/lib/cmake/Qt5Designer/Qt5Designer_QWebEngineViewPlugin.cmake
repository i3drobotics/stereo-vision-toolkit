
add_library(Qt5::QWebEngineViewPlugin MODULE IMPORTED)


_populate_Designer_plugin_properties(QWebEngineViewPlugin RELEASE "designer/qwebengineview.dll" TRUE)

list(APPEND Qt5Designer_PLUGINS Qt5::QWebEngineViewPlugin)
set_property(TARGET Qt5::Designer APPEND PROPERTY QT_ALL_PLUGINS_designer Qt5::QWebEngineViewPlugin)
set_property(TARGET Qt5::QWebEngineViewPlugin PROPERTY QT_PLUGIN_TYPE "designer")
set_property(TARGET Qt5::QWebEngineViewPlugin PROPERTY QT_PLUGIN_EXTENDS "")
