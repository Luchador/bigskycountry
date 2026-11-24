# Seed an editable runtime config, without overwriting the user's preferences.
if(NOT EXISTS "${destination}")
    get_filename_component(directory "${destination}" DIRECTORY)
    file(MAKE_DIRECTORY "${directory}")
    file(COPY_FILE "${source}" "${destination}")
endif()
