#!/bin/bash
# cam-integrazione.sh (agente cam6; 05/11/2026, rifatto 06/11): unisce i rami fotocamera nel ramo di integrazione
# codex/cam-int-20261106 (da f30e206, contiene gia' l'ABI v2 e lo scheletro GuampCamPipe), importa guamp-isp
# (repo a parte) in package/guamp-isp, controlla conflitti, sintassi (~/cc-nld.sh), test host (sempre: camios, campipe,
# g3a, camera, camss), script Python e applicazione delle patch XNU; scrive un rapporto.
# Solo commit locali: NON fa push, NON lancia CI, NON tocca il telefono.
#
#   ~/moto/cam-integrazione.sh [--dry] [--stop] [--reset] [ramo ...]
#     senza rami: ordine fisso (sotto) + qualsiasi altro refs/heads/codex/cam-*-$CAM_DATE trovato
#     --dry    elenca rami e commit, non unisce
#     --stop   al primo conflitto si ferma (lascia il merge aperto nel worktree per risolverlo a mano)
#     --reset  ricrea il ramo di integrazione da CAM_BASE (ATTENZIONE: perde i commit ABI/scheletro: usarlo solo con
#              un CAM_INT diverso da codex/cam-int-20261106)
#   Variabili: CAM_DATE (20261106), CAM_BASE (f30e206), CAM_INT (codex/cam-int-20261106), CAM_WT (~/moto/wt-cam-int-1106),
#              CAM_REPO (~/moto/UniXNU-guamp), CAM_REPORT (rapporto in analisi/camera/out),
#              CAM_ISP_REPO (~/moto/guamp-isp), CAM_ISP_REF (codex/isp-$CAM_DATE; se manca, master)
set -u
export LC_ALL=C
REPO=${CAM_REPO:-$HOME/moto/UniXNU-guamp}
DATE=${CAM_DATE:-20261106}
BASE=${CAM_BASE:-f30e206}
INT=${CAM_INT:-codex/cam-int-20261106}
WT=${CAM_WT:-$HOME/moto/wt-cam-int-1106}
ISPREPO=${CAM_ISP_REPO:-$HOME/moto/guamp-isp}
ISPREF=${CAM_ISP_REF:-codex/isp-$DATE}
WIN=/mnt/c/Users/Chris/Desktop/Moto/analisi/camera/out
REPORT=${CAM_REPORT:-$WIN/cam-integrazione-rapporto.txt}
CCNLD=$HOME/cc-nld.sh
XNUREF=$HOME/moto/xnu-ref
# Ordine dei merge (cam6-integrazione.md §2.1): prima ciò che non tocca l'hardware, poi dal basso verso l'alto.
# guamp-isp (agente 3) non e' un ramo di questo repo: viene importato dopo i rami del kernel (passo "isp" sotto).
ORDER=(
  codex/cam-sensori-20261105      # agente 1: sensori (GuampCamera.inc, GuampCameraSensor.h; il ramo resta quello del 05/11)
  codex/cam-sensori-$DATE         # agente 1, se apre un ramo nuovo
  codex/cam-camss-$DATE           # agente 2: CSIPHY/CSID/TFE RDI, CPAS, SMMU (GuampCamss*.h/.inc)
  codex/cam-3a-$DATE              # agente 4: lenti, EEPROM AF, flash (GuampCamera3A*, GuampTorch*), package/GuampCamera3A
  codex/cam-ios-$DATE             # agente 5: servizio iOS (GuampCamIos.inc, package/GuampISP)
  codex/cam-secondarie-$DATE      # agente 7: OV02B depth/macro (nome proposto)
  codex/cam-video-$DATE           # agente 8: video e salvataggio (nome proposto)
)
DRY=0; STOP=0; RESET=0; ARGS=()
for a in "$@"; do
  case "$a" in --dry) DRY=1 ;; --stop) STOP=1 ;; --reset) RESET=1 ;; -h|--help) sed -n 2,14p "$0"; exit 0 ;; *) ARGS+=("$a") ;; esac
done
cd "$REPO" || exit 2
if ((${#ARGS[@]})); then BRANCHES=("${ARGS[@]}"); else
  BRANCHES=()
  for b in "${ORDER[@]}"; do git rev-parse -q --verify "refs/heads/$b" >/dev/null && BRANCHES+=("$b"); done
  while read -r b; do
    [[ "$b" == "$INT" ]] && continue
    [[ " ${BRANCHES[*]} " == *" $b "* ]] && continue
    BRANCHES+=("$b")
  done < <(git for-each-ref --format='%(refname:short)' "refs/heads/codex/cam-*-$DATE" | sort)
fi
mkdir -p "$(dirname "$REPORT")"
LOG=$(mktemp); trap 'rm -f "$LOG"' EXIT
say() { echo "$*" | tee -a "$LOG"; }
PROBLEMI=0
say "== cam-integrazione $(date '+%F %T')  base $BASE  ramo $INT  worktree $WT"
say "-- rami (${#BRANCHES[@]}):"
for b in "${BRANCHES[@]}"; do
  if ! git rev-parse -q --verify "refs/heads/$b" >/dev/null; then say "   $b: NON ESISTE"; PROBLEMI=$((PROBLEMI+1)); continue; fi
  n=$(git rev-list --count "$BASE..$b"); base_ok=$(git merge-base --is-ancestor "$BASE" "$b" && echo "da $BASE" || echo "NON contiene $BASE")
  wtp=$(git worktree list --porcelain | awk -v r="refs/heads/$b" '/^worktree /{w=$2} $0=="branch "r{print w}')
  dirty=""; nd=0; [[ -n "$wtp" ]] && nd=$(git -C "$wtp" status --porcelain | grep -c .); ((nd)) && dirty="  ATTENZIONE: $nd file non committati/non tracciati in $wtp (non entrano)"
  say "   $b: $n commit, $base_ok, $(git log -1 --format='%h %s' "$b" | cut -c1-90)$dirty"
done
((DRY)) && { cp "$LOG" "$REPORT"; exit 0; }

# worktree di integrazione
if ((RESET)) || ! git rev-parse -q --verify "refs/heads/$INT" >/dev/null; then
  [[ -d "$WT" ]] && git worktree remove --force "$WT"
  git branch -f "$INT" "$BASE" >/dev/null || exit 2
fi
[[ -d "$WT" ]] || git worktree add -q "$WT" "$INT" || exit 2
cd "$WT" || exit 2
if [[ -f .git/MERGE_HEAD || -n "$(git rev-parse -q --verify MERGE_HEAD 2>/dev/null)" ]]; then
  say "merge ancora aperto in $WT: risolvilo o 'git merge --abort', poi rilancia"; cp "$LOG" "$REPORT"; exit 3
fi
[[ -n "$(git status --porcelain --untracked-files=no)" ]] && { say "worktree $WT sporco: mi fermo"; cp "$LOG" "$REPORT"; exit 3; }

say "-- merge"
UNITI=(); CONFLITTI=()
for b in "${BRANCHES[@]}"; do
  git rev-parse -q --verify "refs/heads/$b" >/dev/null || continue
  if git merge-base --is-ancestor "$b" HEAD; then say "   $b: già dentro (nessun commit nuovo)"; continue; fi
  if git merge --no-ff --no-edit -m "Merge branch '$b' into $INT" "$b" >/dev/null 2>>"$LOG"; then
    say "   $b: unito ($(git rev-parse --short HEAD))"; UNITI+=("$b")
  else
    files=$(git diff --name-only --diff-filter=U | tr '\n' ' ')
    say "   $b: CONFLITTO in: $files"; CONFLITTI+=("$b: $files"); PROBLEMI=$((PROBLEMI+1))
    if ((STOP)); then say "   --stop: merge lasciato aperto in $WT"; cp "$LOG" "$REPORT"; exit 1; fi
    git merge --abort
  fi
done

# guamp-isp (agente 3): i file tracciati di $ISPREF (o master) in package/guamp-isp, un commit se cambiano
say "-- isp"
if [[ -d "$ISPREPO/.git" ]]; then
  ref=$ISPREF; git -C "$ISPREPO" rev-parse -q --verify "refs/heads/$ref" >/dev/null || ref=master
  sha=$(git -C "$ISPREPO" rev-parse --short "$ref")
  [[ -n "$(git -C "$ISPREPO" status --porcelain --untracked-files=no)" ]] && say "   ATTENZIONE: $ISPREPO ha modifiche non committate (non entrano)"
  git rm -r -q --ignore-unmatch package/guamp-isp
  mkdir -p package/guamp-isp
  git -C "$ISPREPO" archive "$ref" | tar -x -C package/guamp-isp
  git add -A package/guamp-isp
  if git diff --cached --quiet; then say "   guamp-isp $ref $sha: invariato"; else
    git commit -q -m "package/guamp-isp: software ISP gisp imported from ~/moto/guamp-isp $ref $sha (agent 3)" &&
      say "   guamp-isp $ref $sha: importato ($(git rev-parse --short HEAD))"; fi
else say "   $ISPREPO assente: package/guamp-isp invariato"; fi

say "-- controlli su $BASE..HEAD"
CHANGED=$(git diff --name-only "$BASE" HEAD)
say "   file cambiati: $(echo "$CHANGED" | grep -c .)"
# 1. marcatori di conflitto e spazi
if git grep -n -E '^(<<<<<<<|>>>>>>>) ' -- $(echo "$CHANGED" | grep -v '\.patch$') >>"$LOG" 2>/dev/null; then
  say "   marcatori di conflitto: TROVATI"; PROBLEMI=$((PROBLEMI+1)); else say "   marcatori di conflitto: nessuno"; fi
ws=$(git diff --check "$BASE" HEAD -- . ':!*.patch' 2>&1 | grep -c ':[0-9]*:')
say "   spazi a fine riga/errori diff --check (esclusi .patch): $ws"
# 2. sintassi C++ kernel: ogni .cpp toccato, e il .cpp che include i .inc/.h toccati (NativeEmmc.cpp)
declare -A TU=()
while read -r f; do
  [[ -z "$f" || ! -f "$f" ]] && continue
  case "$f" in
    drivers/*/tests/*) ;;
    drivers/*.cpp) TU[$f]=1 ;;
    drivers/*.inc|drivers/*.h) d=$(dirname "$f"); for c in "$d"/*.cpp; do grep -q "$(basename "$f")" "$c" 2>/dev/null && TU[$c]=1; done ;;
  esac
done <<<"$CHANGED"
[[ -f drivers/NativeEmmc/NativeEmmc.cpp ]] && TU[drivers/NativeEmmc/NativeEmmc.cpp]=1
for c in "${!TU[@]}"; do
  out=$("$CCNLD" "$WT/$c" 2>&1)
  if [[ -z "$out" ]]; then say "   cc-nld $c: OK"; else
    say "   cc-nld $c: ERRORI ($(echo "$out" | grep -c 'error:') error, $(echo "$out" | grep -c 'warning:') warning)"
    echo "$out" | grep -E 'error:|warning:' | head -20 | sed 's/^/      /' | tee -a "$LOG" >/dev/null; PROBLEMI=$((PROBLEMI+1)); fi
done
# 3. test host dei driver (run-*-tests.sh nelle cartelle toccate)
for t in $(echo "$CHANGED" | grep -E '^drivers/[^/]+/' | cut -d/ -f1-2 | sort -u); do
  for s in "$t"/tests/run-*-tests.sh; do
    [[ -f "$s" ]] || continue
    if echo "$CHANGED" | grep -q -E "$(basename "$s" | sed 's/run-\(.*\)-tests.sh/\1/I')" -i; then
      if (cd "$(dirname "$s")" && bash "./$(basename "$s")") >"$LOG.t" 2>&1; then say "   test $s: PASS"; else
        say "   test $s: FALLITO"; tail -15 "$LOG.t" | sed 's/^/      /' | tee -a "$LOG" >/dev/null; PROBLEMI=$((PROBLEMI+1)); fi
      rm -f "$LOG.t"
    fi
  done
done
# 3b. test dell'intera catena, sempre (ABI v2 e scheletro: un pezzo nuovo di solito si rompe qui)
for s in drivers/NativeEmmc/tests/run-camios-tests.sh drivers/NativeEmmc/tests/run-campipe-tests.sh \
         drivers/NativeEmmc/tests/run-camss-tests.sh package/GuampCamera3A/tests/run-g3a-tests.sh; do
  [[ -f "$s" ]] || continue
  if (cd "$(dirname "$s")" && bash "./$(basename "$s")") >"$LOG.t" 2>&1; then say "   catena $s: PASS"; else
    say "   catena $s: FALLITO"; tail -15 "$LOG.t" | sed 's/^/      /' | tee -a "$LOG" >/dev/null; PROBLEMI=$((PROBLEMI+1)); fi
done
GCAM=/mnt/c/Users/Chris/Desktop/Moto/analisi/camera/out/guamp-camera-v2.gcam
if [[ -f drivers/NativeEmmc/tests/run-camera-tests.sh ]]; then
  if (cd drivers/NativeEmmc/tests && bash ./run-camera-tests.sh $([[ -f $GCAM ]] && echo "$GCAM")) >"$LOG.t" 2>&1; then
    say "   catena run-camera-tests.sh: PASS"; else
    say "   catena run-camera-tests.sh: FALLITO"; tail -15 "$LOG.t" | sed 's/^/      /' | tee -a "$LOG" >/dev/null; PROBLEMI=$((PROBLEMI+1)); fi
fi
# 4. script Python
for f in $(echo "$CHANGED" | grep '\.py$'); do
  [[ -f "$f" ]] || continue
  if python3 -I -c "import ast,sys; ast.parse(open(sys.argv[1]).read(), sys.argv[1])" "$f" 2>>"$LOG"; then :; else
    say "   python $f: ERRORE DI SINTASSI"; PROBLEMI=$((PROBLEMI+1)); fi
done
say "   python: $(echo "$CHANGED" | grep -c '\.py$') file controllati"
# 5. patch XNU: applicazione in ordine su una copia di xnu-ref (come sim.sh)
if echo "$CHANGED" | grep -q -E '^(patches/xnu|kits/NativePrelink/patches)/.*\.patch$' && [[ -d "$XNUREF/.git" ]]; then
  SIM=$(mktemp -d); git -c advice.detachedHead=false clone -q --shared "$XNUREF" "$SIM/x" 2>/dev/null && cd "$SIM/x"
  n=0; bad=0
  for P in "$WT"/patches/xnu/*.patch "$WT"/kits/NativePrelink/patches/*.patch; do
    [[ -f "$P" ]] || continue
    if ! git apply --reverse --check "$P" >/dev/null 2>&1; then
      if git apply "$P" 2>>"$LOG"; then n=$((n+1)); else say "   patch FALLITA: ${P#$WT/}"; bad=$((bad+1)); fi
    fi
  done
  cd "$WT"; rm -rf "$SIM"
  say "   patch XNU applicate in sequenza: $n, fallite: $bad"; PROBLEMI=$((PROBLEMI+bad))
fi

say "-- esito: $INT @ $(git rev-parse --short HEAD), uniti ${#UNITI[@]}, conflitti ${#CONFLITTI[@]}, problemi $PROBLEMI"
for c in "${CONFLITTI[@]}"; do say "   conflitto: $c"; done
say "   NON pushato. Per il push/CI: decide il coordinatore."
cp "$LOG" "$REPORT"
echo "rapporto: $REPORT"
((PROBLEMI)) && exit 1 || exit 0
