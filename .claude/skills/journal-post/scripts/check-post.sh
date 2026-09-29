#!/usr/bin/env bash
# Vérifie un article du journal et son dossier de médias avant commit.
#
# Usage : check-post.sh DOCS/_posts/AAAA-MM-JJ-slug.md
#
# - construit le site (erreurs Jekyll / Liquid)
# - chaque image ou vidéo référencée existe dans le site construit
# - aucun fichier du dossier de médias n'est inutilisé
# - aucune métadonnée EXIF/GPS dans les photos, aucun tag de localisation
#   dans les vidéos
set -uo pipefail

post=$(realpath "$1")
root=$(git -C "$(dirname "$post")" rev-parse --show-toplevel)
slug=$(basename "$post" .md)
media="$root/DOCS/assets/img/$slug"
status=0

cd "$root/DOCS"
if bundle exec jekyll build 2>&1 | grep -E "Error|Liquid"; then status=1; fi

page="_site/journal/$slug/index.html"
[ -f "$page" ] || { echo "page non générée : $page"; exit 1; }

for f in $(grep -o '<img src="[^"]*"\|<source src="[^"]*"\|<model-viewer[^>]*src="[^"]*"' "$page" | sed 's/.*src="//;s/"//'); do
  [ -f "_site$f" ] || { echo "MANQUANT : $f"; status=1; }
done

if [ -d "$media" ]; then
  for f in "$media"/*; do
    [ -e "$f" ] || continue
    grep -q "$(basename "$f")" "$post" || { echo "non utilisé : $(basename "$f")"; status=1; }
    case "${f,,}" in
      *.jpg|*.jpeg|*.png)
        n=$(identify -verbose "$f" 2>/dev/null | grep -ci 'exif:\|gps')
        [ "$n" -eq 0 ] || { echo "métadonnées restantes : $(basename "$f")"; status=1; } ;;
      *.mp4|*.mov)
        n=$(ffprobe -v error -show_entries format_tags -of default=nw=1 "$f" | grep -ci 'location')
        [ "$n" -eq 0 ] || { echo "GPS dans la vidéo : $(basename "$f")"; status=1; } ;;
    esac
  done
  echo "médias : $(ls "$media" | wc -l) fichiers, $(du -sh "$media" | cut -f1)"
fi

echo "images : $(grep -c '<img src' "$page")  vidéos : $(grep -c '<video' "$page")  modèles 3D : $(grep -c '<model-viewer' "$page")"
[ $status -eq 0 ] && echo "OK" || echo "PROBLÈMES DÉTECTÉS"
exit $status
