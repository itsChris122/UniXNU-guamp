# Punto d'avvio Wi-Fi Guamp nel kernel Redwood

Verificato il 28 ottobre 2026 sui sorgenti locali HackiOS, senza modificarli.

## Riscontri nel codice

- `sorgenti/HackiOS/hackios-xnu-kernel/xnu/files/bsd/dev/redwood_input.c`, `redwood_input_init()`, inizializza eMMC/UFS, poi chiama `redwood_net_init()`, `redwood_adreno_init()` e `redwood_wifi_init()`.
- `redwood_net_init()` in `bsd/dev/redwood_net.c` registra solo il link virtio. Non esiste un callback generico che scopra o avvii il link SNOC Guamp.
- `redwood_net_attach()` crea `en0`; per un link wireless chiama `RedwoodWLANStart("en0", address)`, che pubblica il servizio IOKit atteso dal lato iOS. Perci? l'attach del link deve succedere dopo l'inizializzazione BSD della rete.
- `NativeEmmc` avvia `wifiprep::start()` su un thread separato. Il BDF viene letto e conservato da `FirmwareStore`; il callback firmware pu? restituire `NotReady` finch? quel prep non ? terminato.
- `guamp_xnu_wifi_attach()` avvia SNOC e attende sincronicamente fino a 10 secondi che ath10k esponga un MAC, poi crea `en0`, collega station e avvia il worker. Non va invocata nel percorso sincrono di `ArmDTPlatform::configure()` o dentro `redwood_input_init()`.

## Sequenza necessaria

1. `ArmDTPlatform` pubblica il device tree e avvia `NativeEmmc`; il prep opt-in monta e convalida il firmware/BDF in sola lettura.
2. Il provider SM6115 acquisisce risorse e ownership per modem, SCM/PAS, RPM, SMEM/GLINK/QRTR, MSA, power, SMMU e CE. Non deve assumere che una mappatura o un context bank sia libero senza averlo verificato e reclamato.
3. Dopo che BSD ha inizializzato `redwood_net` e il provider conferma che il firmware-store ? pronto, un thread kernel dedicato chiama `guamp_xnu_wifi_attach()`. L'attach crea `en0`; il successivo `RedwoodWLANStart` pubblica il lato IOKit.
4. `redwood_sta` riceve in seguito le richieste di scansione e join da iOS; solo dopo associazione e chiavi installate il provider segnala link-up.

## Cosa resta da implementare

Nel kernel Guamp non ? stato trovato un hook equivalente gi? collegato a `guamp_xnu_wifi_attach()`. Serve un adattatore nel kernel HackiOS/Guamp che avvii il worker al momento giusto e costruisca `lx_platform` e `gw_snoc_board` con le risorse reali. Il provider hardware non esiste ancora; non ? corretto aggiungere un attach anticipato o un callback di successo fittizio.

Il bridge attuale include `redwood_sta`/`redwood_net` dal checkout HackiOS; questi header e le implementazioni non sono nel package kernel Guamp. Finch? l'adattatore non viene integrato e compilato contro il package effettivo, il bridge non ? un driver installabile e WPA2 non ? pronta. Nessuna build o operazione sul telefono ? stata eseguita in questa analisi.
