#!/usr/bin/env bash
# Optimise une photo ou une vidéo du journal pour le web.
#
# Usage : optimize-media.sh <fichier-source> <destination-sans-extension>
#   ex. : optimize-media.sh DOCS/assets/img/2026-09-29-x/IMG_1234.jpg \
#                           DOCS/assets/img/2026-09-29-x/x-driver-dessus
#
# - photos (jpg/jpeg) : 1600 px max, qualité 82, JPEG progressif
# - captures (png)    : 1400 px max
# - vidéos (mp4/mov)  : H.264 720p 30 i/s, CRF 27, son AAC mono 64 kb/s
# - toutes les métadonnées sont effacées (EXIF, GPS, tag « location »)
# - l'original est copié hors du dépôt avant d'être supprimé :
#   ${JOURNAL_BACKUP_DIR:-~/.cache/cdr-journal-originals}/<dossier>/
set -euo pipefail

src=$1
dest=$2
[ -f "$src" ] || { echo "introuvable : $src" >&2; exit 1; }

backup="${JOURNAL_BACKUP_DIR:-$HOME/.cache/cdr-journal-originals}/$(basename "$(dirname "$src")")"
mkdir -p "$backup"
cp --update=none "$src" "$backup/"

ext="${src##*.}"
ext="${ext,,}"
case "$ext" in
  jpg|jpeg)
    out="$dest.jpg"
    convert "$src" -auto-orient -strip -resize "1600x1600>" -quality 82 -interlace Plane "$out.tmp.jpg"
    ;;
  png)
    out="$dest.png"
    convert "$src" -strip -resize "1400x1400>" "$out.tmp.png"
    ;;
  mp4|mov)
    out="$dest.mp4"
    ffmpeg -v error -y -i "$src" -map 0:v:0 -map "0:a:0?" -map_metadata -1 -map_chapters -1 \
      -vf "scale=-2:720,fps=30" -c:v libx264 -preset slow -crf 27 -pix_fmt yuv420p \
      -movflags +faststart -c:a aac -b:a 64k -ac 1 "$out.tmp.mp4"
    ;;
  *)
    echo "format non géré : $src" >&2
    exit 1
    ;;
esac

mv "$out.tmp.${out##*.}" "$out"
[ "$src" -ef "$out" ] || rm "$src"
printf '%s -> %s (%s)\n' "$(basename "$src")" "$(basename "$out")" "$(du -h "$out" | cut -f1)"
