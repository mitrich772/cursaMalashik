cmake_minimum_required(VERSION 3.5)

project(cursaMalashik VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Находим Qt и необходимые модули, включая Charts и Widgets
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets Charts)

# Источники проекта
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(cursaMalashik
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(cursaMalashik SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(cursaMalashik
            ${PROJECT_SOURCES}
            customwidget.h customwidget.cpp
            result.txt
        )
    endif()
endif()

# Подключение библиотек QtWidgets и QtCharts
target_link_libraries(cursaMalashik PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts)

# Настройки для macOS и iOS, если применимо
if(${QT_VERSION_MAJOR} VERSION_LESS 6 AND ${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.cursaMalashik)
endif()

set_target_properties(cursaMalashik PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Установка файлов при сборке
include(GNUInstallDirs)
install(TARGETS cursaMalashik
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Финализация для Qt6
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(cursaMalashik)
endif()
