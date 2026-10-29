#!/usr/bin/env bash
# Checks a staged depth-camera package before it is published.
#
#   Deployment/CheckPackage.sh <package-dir> "<expected backends>"
#   Deployment/CheckPackage.sh build/depth-camera "orbbec freenect k4a"
#
# A package that is merely smaller than it should be is the failure mode to
# worry about here: every backend is optional at configure time, so a missing
# system dependency silently drops one and the build still succeeds. Naming the
# expected set turns that into an error.
set -uo pipefail

DIR=${1:?usage: CheckPackage.sh <package-dir> "<expected backends>"}
EXPECT=${2:?usage: CheckPackage.sh <package-dir> "<expected backends>"}

case "$(uname -s)" in
  Darwin)             EXT=dylib ;;
  MINGW*|MSYS*|CYGWIN*) EXT=dll  ;;
  *)                  EXT=so    ;;
esac

fail=0
note() { echo "::error::$*"; fail=1; }

[[ -d "$DIR" ]] || { note "no package directory at $DIR"; exit 1; }

echo "--- $DIR"
ls -la "$DIR"

### 1. Every backend the platform is meant to ship is there, and nothing else.
for b in $EXPECT; do
  f="$DIR/score_depthcam_$b.$EXT"
  if [[ -f "$f" ]]; then
    echo "  ok    score_depthcam_$b.$EXT ($(du -h "$f" | cut -f1))"
  else
    note "missing backend: score_depthcam_$b.$EXT"
  fi
done

for f in "$DIR"/score_depthcam_*."$EXT"; do
  [[ -e "$f" ]] || continue
  name=$(basename "$f" ".$EXT"); name=${name#score_depthcam_}
  grep -qw -- "$name" <<< "$EXPECT" || note "unexpected backend in package: $name"
done

### 2. One exported symbol each.
###
### The whole design rests on this: score dlopens everything in its package and
### support directories, on Linux with RTLD_GLOBAL, so a backend that exports
### anything beyond its entry point can interpose a sibling's libusb or libjpeg.
### Five SDKs, five private copies.
###
### Windows is checked differently, and less strictly. A PE has no global symbol
### namespace -- imports bind per module, by DLL name -- so the interposition
### this guards against cannot happen there. It is also not achievable: both
### libfreenect and librealsense mark their public API __declspec(dllexport)
### unconditionally in headers we do not own, and neither a .def file nor a
### linker flag removes an export the compiler put there. So on Windows the
### requirement is that the entry point is present, and the count is reported.
for f in "$DIR"/score_depthcam_*."$EXT"; do
  [[ -e "$f" ]] || continue
  case "$EXT" in
    so)    syms=$(nm -D --defined-only "$f" | wc -l) ;;
    # Apple's nm is llvm-nm: -g is external symbols, -U is --defined-only.
    # Distinct symbol names, not lines: on a universal binary nm reports each
    # slice in turn with a header and a blank line, so a correct fat module
    # with one export per slice would count as six lines.
    dylib) syms=$(nm -gU "$f" 2>/dev/null | awk 'NF == 3 { print $3 }' | sort -u | wc -l) ;;
    dll)
      command -v dumpbin >/dev/null || { echo "  --    $(basename "$f"): no dumpbin, exports unchecked"; continue; }
      # dumpbin's export rows are "ordinal hint RVA name" and nothing else in
      # its output has that shape; a looser match picks up the header and the
      # summary and reports extras that are not there.
      all=$(dumpbin //exports "$f" \
            | awk 'NF==4 && $1 ~ /^[0-9]+$/ && $3 ~ /^[0-9A-Fa-f]{8}$/ {print $4}')
      if grep -qx "score_depthcam_backend_v1" <<< "$all"; then
        n=$(wc -l <<< "$all")
        if [[ "$n" -eq 1 ]]; then
          echo "  ok    $(basename "$f") exports 1 symbol"
        else
          echo "  ok    $(basename "$f") exports score_depthcam_backend_v1 (+ $((n - 1)) from the SDK's own dllexport)"
        fi
      else
        note "$(basename "$f") does not export score_depthcam_backend_v1"
      fi
      continue
      ;;
    *)     continue ;;
  esac
  if [[ "$syms" -ne 1 ]]; then
    note "$(basename "$f") exports $syms symbols, expected exactly 1"
    [[ "$EXT" == so ]] && nm -D --defined-only "$f" | head -20
    [[ "$EXT" == dylib ]] && nm -gU "$f" | head -20
    [[ "$EXT" == dll ]] && dumpbin //exports "$f" | head -30
  else
    echo "  ok    $(basename "$f") exports 1 symbol"
  fi
done

### 3. Every library a backend imports is either the platform's or in the package.
###
### A backend that links a dependency dynamically by accident still builds,
### still passes the symbol check above and loads fine on the machine that
### built it, where the library sits in some build tree on the search path --
### and fails with "module not found" everywhere else. Windows reports that
### as a bare error 126, with nothing naming the missing DLL.
###
### System means: on Windows, present in System32 (which covers api-ms-win-*
### forwarders and everything else the OS ships) or the Visual C++
### redistributable the DepthAI backend requires; on macOS, /usr/lib or
### /System; on Linux, a versioned soname the loader resolves outside the
### package -- the package's own libraries are found through $ORIGIN.
for f in "$DIR"/score_depthcam_*."$EXT"; do
  [[ -e "$f" ]] || continue
  bad=""
  case "$EXT" in
    so)
      # An unversioned soname (libusb-1.0.so) is what only a -dev package
      # installs, so the system copy on the build machine does not count.
      notfound=$(ldd "$f" 2>/dev/null | awk '/not found/ { print $1 }')
      while read -r lib; do
        [[ -n "$lib" ]] || continue
        [[ -e "$DIR/$lib" ]] && continue
        if [[ "$lib" == *.so ]] || grep -qxF -- "$lib" <<< "$notfound"; then
          bad="$bad $lib"
        fi
      done < <(readelf -dW "$f" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p')
      ;;
    dylib)
      while read -r dep; do
        case "$dep" in
          @rpath/*|@loader_path/*)
            [[ -e "$DIR/${dep#*/}" ]] || bad="$bad $dep" ;;
          /usr/lib/*|/System/*) ;;
          *) bad="$bad $dep" ;;
        esac
      done < <(otool -L "$f" | tail -n +2 | awk '{ print $1 }' | grep -v "$(basename "$f")")
      ;;
    dll)
      command -v dumpbin >/dev/null || continue
      # Both the import table and the delay-load one: a delay-loaded DLL that
      # is not shipped fails later, on first call, instead of at load.
      while read -r dll; do
        [[ -e "$DIR/$dll" ]] && continue
        [[ -e "/c/Windows/System32/$dll" ]] && continue
        lower=$(tr '[:upper:]' '[:lower:]' <<< "$dll")
        case "$lower" in
          api-ms-win-*|ext-ms-*) continue ;;
          msvcp140*.dll|vcruntime140*.dll|concrt140.dll|vcomp140.dll) continue ;;
        esac
        bad="$bad $dll"
      done < <(dumpbin //dependents "$f" | tr -d '\r' \
               | awk '/following (delay load )?dependencies:/ { on=1; next }
                      /^ *Summary/ { on=0 }
                      on && $1 ~ /\.[dD][lL][lL]$/ { print $1 }')
      ;;
  esac
  if [[ -n "$bad" ]]; then
    note "$(basename "$f") imports libraries that are neither system nor in the package:$bad"
  else
    echo "  ok    $(basename "$f") imports nothing outside the system and the package"
  fi
done

### 4. The manifest score reads after installing.
###
### Checked with grep rather than a JSON parser on purpose: this runs inside Git
### Bash on the Windows runner, where `python3` is a Microsoft Store stub that
### prints an advert and exits 9009.
if [[ -f "$DIR/package.json" ]]; then
  missing=""
  for k in key raw_name name version kind architecture; do
    grep -q "\"$k\"" "$DIR/package.json" || missing="$missing $k"
  done
  if [[ -n "$missing" ]]; then
    note "package.json is missing:$missing"
  else
    echo "  ok    package.json: $(sed -n 's/.*"architecture" *: *"\([^"]*\)".*/\1/p' "$DIR/package.json")"
  fi
else
  note "no package.json in the package"
fi

exit $fail
