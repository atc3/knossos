find_package(Git)
if(Git_FOUND AND EXISTS ${GIT}/.git)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --always --dirty --tags
        OUTPUT_VARIABLE KREVISION
        WORKING_DIRECTORY ${GIT}
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%aI
        OUTPUT_VARIABLE KREVISIONDATE
        WORKING_DIRECTORY ${GIT}
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # branch name, so it is obvious from the title bar which build is open
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE KBRANCH
        WORKING_DIRECTORY ${GIT}
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(KBRANCH STREQUAL "HEAD")
        set(KBRANCH "detached")
    endif()
endif()
if(NOT KREVISION)
    set(KREVISION "5.1")
    message("couldn’t get version from git, setting to ${KREVISION}")
else()
    message("building ${KREVISION} on branch ${KBRANCH}")
endif()
configure_file(${SRC} ${DST} @ONLY)
