My project Big Sky Country

## World saves

The editor and game open `world.wld` in the SDL per-user data directory by default. On Windows the path is `%APPDATA%\BigSkyCountry\BigSkyCountry\world.wld`. To work on a different world, pass its path as the first argument to both executables.

```powershell
.\BigSkyCountryEditor.exe .\montana.wld
.\BigSkyCountry.exe .\montana.wld
```

A missing world starts empty. The editor writes it on the first save. Finishing a road, editing a committed road, and undoing or redoing a road edit all save the world. File > Save (Ctrl+S) also writes it. The old `.roads` format is retired.

### WLD version 1

All framing integers are unsigned and little-endian. The file begins with the four ASCII bytes `BSCW`, a 32-bit format version, and a 32-bit section count. Each section has a four-byte tag, a 32-bit section version, a 64-bit payload length in bytes, and exactly that many payload bytes. Sections are independent of their physical order.

`ROAD` version 1 stores roads in draw order. Its payload is whitespace-separated text in the classic locale. Each road starts with lane count, curved flag, left and right shoulder flags, three-lane layout, divided flag, and divided gap. Next comes the point count, then one record per point with X, Y, Z, a manual-heading flag, and a heading in radians. Flags are 0 or 1. The reader rebuilds the meshes from these records. An absent or empty `ROAD` section means the world has no roads.

To add terrain or objects, give the new data its own tag and section version. The current editor preserves unrecognized sections byte-for-byte when saving road edits. It rejects unsupported versions of the world container or the known `ROAD` section.
