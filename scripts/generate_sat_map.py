#!/usr/bin/env python3
"""
generate_sat_map.py

Fetches map tiles centered at a given latitude/longitude and stitches them into
one image suitable as a Gazebo ground texture.

Usage:
  python3 generate_sat_map.py --lat 52.0 --lon 5.0 --zoom 16 --tiles 5

Defaults use ESRI World Imagery as tile source (satellite) which is commonly
available. If you prefer another provider, pass --tile-url with a format string
containing {z}, {x}, {y} (e.g. 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png').

Note: Respect the tile provider's terms of use.

Requires: pillow, requests
Install: pip3 install pillow requests
"""

import os
import math
import argparse
import io
import sys
from PIL import Image
import requests

TILE_SERVERS = {
    'esri': 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    'osm': 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png',
}

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'media', 'materials')
TEXTURE_DIR = os.path.join(OUT_DIR, 'textures')
SCRIPTS_DIR = os.path.join(OUT_DIR, 'scripts')

os.makedirs(TEXTURE_DIR, exist_ok=True)
os.makedirs(SCRIPTS_DIR, exist_ok=True)


def latlon_to_tile(lat_deg, lon_deg, zoom):
    """Return tile x,y indexes for given lat/lon at zoom (Web Mercator)."""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    x_tile = int((lon_deg + 180.0) / 360.0 * n)
    y_tile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return x_tile, y_tile


def fetch_tile(url):
    resp = requests.get(url, timeout=10)
    resp.raise_for_status()
    return Image.open(io.BytesIO(resp.content)).convert('RGB')


def stitch_tiles(center_x, center_y, zoom, tiles_across, tile_url):
    half = tiles_across // 2
    tile_imgs = []
    for dy in range(-half, half + 1):
        row = []
        for dx in range(-half, half + 1):
            x = center_x + dx
            y = center_y + dy
            url = tile_url.format(z=zoom, x=x, y=y)
            try:
                img = fetch_tile(url)
            except Exception as e:
                print(f"Warning: failed to fetch tile {x},{y} -> {e}", file=sys.stderr)
                # create empty tile placeholder
                img = Image.new('RGB', (256, 256), (128, 128, 128))
            row.append(img)
        tile_imgs.append(row)

    tile_w, tile_h = tile_imgs[0][0].size
    out_w = tile_w * tiles_across
    out_h = tile_h * tiles_across
    out = Image.new('RGB', (out_w, out_h))

    for ry, row in enumerate(tile_imgs):
        for rx, img in enumerate(row):
            out.paste(img, (rx * tile_w, ry * tile_h))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--lat', type=float, required=True)
    p.add_argument('--lon', type=float, required=True)
    p.add_argument('--zoom', type=int, default=16)
    p.add_argument('--tiles', type=int, default=5, help='odd number of tiles across (e.g. 3,5,7)')
    p.add_argument('--tile-server', choices=TILE_SERVERS.keys(), default='esri')
    p.add_argument('--tile-url', default=None, help='override tile URL template using {z},{x},{y}')
    p.add_argument('--out', default=os.path.join(TEXTURE_DIR, 'map.png'))
    args = p.parse_args()

    if args.tiles % 2 == 0:
        raise SystemExit('tiles must be an odd number')

    tile_url = args.tile_url if args.tile_url else TILE_SERVERS[args.tile_server]

    cx, cy = latlon_to_tile(args.lat, args.lon, args.zoom)
    print(f'Center tile: {cx},{cy} at zoom {args.zoom}')
    print(f'Fetching {args.tiles}x{args.tiles} tiles from {tile_url}')

    stitched = stitch_tiles(cx, cy, args.zoom, args.tiles, tile_url)

    # Optionally resize to power-of-two or a specific size Gazebo prefers.
    # We'll save at stitched size (tiles*256); users can resize later if needed.
    out_path = os.path.abspath(args.out)
    stitched.save(out_path)
    print(f'Saved texture to {out_path}')

    # Create a simple material script next to the texture so Gazebo can find it.
    mat_path = os.path.join(SCRIPTS_DIR, 'map.material')
    rel_tex = os.path.relpath(out_path, os.path.dirname(mat_path))
    mat_contents = f"""
material My/Map
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {os.path.basename(out_path)}
      }}
    }}
  }}
}}
"""
    with open(mat_path, 'w') as f:
        f.write(mat_contents)
    print(f'Wrote material script to {mat_path}')


if __name__ == '__main__':
    main()
