"""Non-Windows host counterpart to texconv. Write a 2D DX10 DDS with all mips."""
from pathlib import Path
import struct
import sys

import ispc_texcomp as bc
import numpy as np
from PIL import Image, ImageOps


def resize_channels(data, size):
    return np.stack([np.asarray(Image.fromarray(channel).resize(size, Image.Resampling.BOX))
                     for channel in data.transpose(2, 0, 1)], axis=-1)


def encode(data, fmt):
    if fmt == "BC7_UNORM_SRGB":
        # Average colors in linear light and encode them back into sRGB for BC7.
        rgb = data[:, :, :3]
        rgb = np.where(rgb <= 0.0031308, rgb * 12.92, 1.055 * rgb ** (1 / 2.4) - 0.055)
        data = np.concatenate((rgb, data[:, :, 3:4]), axis=2)
    elif fmt == "BC5_UNORM":
        data = data[:, :, :2]
    elif fmt == "BC4_UNORM":
        data = data[:, :, :1]
    h, w, channels = data.shape
    pixels = np.rint(np.clip(data, 0, 1) * 255).astype(np.uint8)
    pixels = np.pad(pixels, ((0, -h % 4), (0, -w % 4), (0, 0)), mode="edge")
    h, w = pixels.shape[:2]
    surface = bc.RGBASurface(pixels.tobytes(), w, h, w * channels)
    if channels == 4:
        profile = "alpha_basic" if np.any(pixels[:, :, 3] != 255) else "basic"
        return bc.compress_blocks_bc7(surface, bc.BC7EncSettings.from_profile(profile))
    if channels == 2:
        return bc.compress_blocks_bc5(surface)
    return bc.compress_blocks_bc4(surface)


def source_levels(source, fmt):
    if Path(source).suffix.lower() == ".dds":
        # BigSkyFoliageMips already filtered and coverage-corrected every level.
        # Pillow opens only level zero, so read our tightly packed DX10 input
        # directly and preserve its chain through block compression.
        raw = Path(source).read_bytes()
        if len(raw) < 148:
            raise ValueError("Truncated foliage mip source")
        header = struct.unpack_from("<37I", raw)
        h, w, count, dxgi = header[3], header[4], header[7], header[32]
        channels = 4 if fmt == "BC7_UNORM_SRGB" else 1
        expected = 29 if channels == 4 else 61
        if (header[0] != 0x20534444 or header[1] != 124 or header[21] != 0x30315844
                or fmt not in ("BC7_UNORM_SRGB", "BC4_UNORM") or dxgi != expected
                or header[33:37] != (3, 0, 1, 0) or not 0 < w <= 16384 or not 0 < h <= 16384
                or count != max(w, h).bit_length() or header[5] != w * channels):
            raise ValueError("Invalid foliage mip source")
        offset = 148
        for _ in range(count):
            size = w * h * channels
            if offset + size > len(raw):
                raise ValueError("Truncated foliage mip level")
            data = np.frombuffer(raw, dtype=np.uint8, count=size, offset=offset).reshape(h, w, channels).astype(np.float32) / 255
            if channels == 4:
                rgb = data[:, :, :3]
                data[:, :, :3] = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
            yield data
            offset += size
            w, h = max(1, w // 2), max(1, h // 2)
        if offset != len(raw):
            raise ValueError("Unexpected data after foliage mip chain")
        return
    yield from image_levels(source, fmt)


def image_levels(source, fmt):
    """Ordinary textures keep their existing filtering path."""
    with Image.open(source) as image:
        image = ImageOps.exif_transpose(image)
        if image.mode.startswith("I;16"):
            # Pillow's direct RGBA conversion clips 16-bit grayscale values at 255.
            grey = np.asarray(image, dtype=np.float32) / 65535
            data = np.stack((grey, grey, grey, np.ones_like(grey)), axis=-1)
        else:
            data = np.asarray(image.convert("RGBA"), dtype=np.float32) / 255
    if fmt == "BC7_UNORM_SRGB":
        rgb = data[:, :, :3]
        data[:, :, :3] = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
    h, w = data.shape[:2]
    while True:
        yield data
        if w == 1 and h == 1:
            return
        w, h = max(1, w // 2), max(1, h // 2)
        data = resize_channels(data, (w, h))


def cook(source, output, fmt):
    levels = source_levels(source, fmt)
    data = next(levels)
    h, w = data.shape[:2]
    mips = max(w, h).bit_length()
    block_bytes = 8 if fmt == "BC4_UNORM" else 16
    pitch = ((w + 3) // 4) * ((h + 3) // 4) * block_bytes
    dxgi = {"BC7_UNORM": 98, "BC7_UNORM_SRGB": 99, "BC5_UNORM": 83, "BC4_UNORM": 80}[fmt]
    with Path(output).open("wb") as out:
        out.write(b"DDS " + struct.pack("<7I", 124, 0xA1007, h, w, pitch, 0, mips) + bytes(44))
        out.write(struct.pack("<II4s", 32, 4, b"DX10") + bytes(20))
        out.write(struct.pack("<5I", 0x401008 if mips > 1 else 0x1000, 0, 0, 0, 0))
        out.write(struct.pack("<5I", dxgi, 3, 0, 1, 0))
        out.write(encode(data, fmt))
        for data in levels:
            out.write(encode(data, fmt))


if __name__ == "__main__":
    cook(*sys.argv[1:])
