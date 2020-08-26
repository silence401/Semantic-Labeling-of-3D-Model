externalproject_add(ext_mve
    PREFIX          mve
    URL  "/home/silence401/wujinbo/mve.zip"
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${CMAKE_SOURCE_DIR}/elibs/mve
    CONFIGURE_COMMAND ""
    BUILD_COMMAND   make -C libs/mve && make -C libs/util #not platform independent
    BUILD_IN_SOURCE true
    INSTALL_COMMAND ""
)
