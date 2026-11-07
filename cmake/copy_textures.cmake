# file(COPY) preserves timestamps and skips textures that are already up to date.
file(COPY "${textureSource}/" DESTINATION "${textureDestination}")

# If a source texture was renamed or deleted, retire only our previous copy.
if(EXISTS "${textureManifest}")
    file(STRINGS "${textureManifest}" previousTextures ENCODING UTF-8)
    foreach(relativePath IN LISTS previousTextures)
        if(NOT EXISTS "${textureSource}/${relativePath}")
            file(REMOVE "${textureDestination}/${relativePath}")
        endif()
    endforeach()
endif()

file(GLOB_RECURSE textures LIST_DIRECTORIES false RELATIVE "${textureSource}" "${textureSource}/*")
list(JOIN textures "\n" manifestContents)
file(WRITE "${textureManifest}" "${manifestContents}\n")
