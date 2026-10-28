# Contratto di bootstrap SM6115 verso la scansione

Ricavato il 28 ottobre 2026 dai sorgenti mantenuti, senza build o accessi al
dispositivo. Questo documento descrive l'ordine necessario; non certifica un
avvio radio riuscito.

## Ordine richiesto dal codice

| Fase | Chiamata o condizione effettiva |
|---|---|
| Firmware | Montare il `FirmwareStore` persistente prima che GLINK/QRTR possa ricevere richieste QFTP. Le callback esposte accettano solo `wlanmdsp.mbn` e BDF Guamp. Il vecchio `firmwareReport()` monta lo store dopo `modemBoot()`; un bootstrap radio deve anticiparlo. |
| Mappe e RPM | Mappare le risorse fisiche, avviare GLINK RPM, attendere `rpm.online()`, poi completare i voti proxy con gli ACK. Un voto accodato non basta. |
| Preparazione APPS | Con PAS fermo confermato, chiamare `WifiControl::prepare(&arena, allocator, ops, true)` per item SMP2P 435 e descrittore/FIFO GLINK 478/479. Questo deve precedere l'avvio PAS; `Smp2pEdge::start()` azzera lo stato APPS e vieta implicitamente il riuso contro un modem attivo. |
| SMMU e DMA | Reclamare il contesto WLAN mentre il remoto è fermo. `WlanSmmu::start()` richiede tabelle fisiche contigue/coerenti da 130×4096 byte, range XNU valido e `radioStopped=true`; poi inizializzare il singleton `gGuampXnuDma` e installarne gli hook. |
| PAS | Usare il loader autenticato con firmware già verificato. Subito dopo `authenticate()` accettato, chiamare `control.modemStarted(now)` e pompare il controllo fino a SMP2P ready e GLINK/QRTR. Il vecchio `modemBoot()` restituisce `void` e fa soltanto osservazione ready: va esposto un esito e aggiunto l'handoff al provider. |
| WLFW | Il provider implementa `assignMsa`, `power`, `fileSize`, `fileRead`, poi continua `control.step(now)` fino a `control.firmwareReady()`. |
| Board ath10k | Compilare `gw_snoc_board` con finestra WLAN da 8 MiB, `firmware_chip_id=control.wlan.capability.chip` ottenuto dal modem, power/ready fisici e binding WLFW. |
| Attach e worker | Dopo l'inizializzazione BSD della rete, un thread dedicato chiama `guamp_xnu_wifi_attach(platform, board)`. Durante questa chiamata deve continuare il pump provider. Al ritorno riuscito il worker station ne assume la proprietà esclusiva. |
| Scan | Le richieste iOS sono inoltrate da `redwood_sta` a `nw_wifi_scan_ex()`. Il provider non dichiara scan riuscita: solo l'evento ath10k `scan_done` completa la richiesta e i BSS sono letti dal relativo inventario. |

## FirmwareReady e mission mode sono passaggi distinti

Nel client mantenuto `GuampWlfw.inc`, `FirmwareReady` è raggiunto senza tabelle
CE: registrazione, MSA info/assignment/ready, capabilities, BDF, calibrazione e
indicazione firmware 0x21 portano allo stato `WaitFirmware` e poi
`FirmwareReady`. La chiamata `enable()` accetta **solo** questo stato e invia
successivamente configurazione CE e mission mode fino a `Enabled`.

La precedente diagnosi che attribuiva `FirmwareReady` a `wlan_enable()` e
considerava circolare l'attesa era quindi errata rispetto a questo sorgente.
Il provider deve arrivare a `FirmwareReady` durante il bootstrap. Il bridge
`Ath10kWlfwBridge::enable()` ora attende anche tale stato prima di inviare la
configurazione, e non lo confonde con `radioEnabled()`.

## Contratto del pump: un solo proprietario

`lx_rt.c::lx_pump()` chiama sempre `lx_platform.poll()` prima di servire IRQ,
timer e work, anche nei wait annidati del probe ath10k. Per questo i completamenti
CE devono progredire anche quando il probe è fermo in attesa di HTC/WMI.

`gw_snoc_poll_ce()` è stato aggiunto per questo confine: serve i dodici CE e
schedula NAPI, con guardia contro rientro CE, senza entrare nel dispatcher o
nella station. `gw_snoc_step()` usa la stessa funzione. Il provider `poll()`
deve chiamare questa API insieme al pump RPM e `WifiControl`, evitando una
chiamata ricorsiva a `nw_wifi_step()` o `lx_pump()`.

Il provider deve proteggere il rientro nel proprio `control.step()`: i callback
sincroni, in particolare l'attesa degli ACK di potenza, possono dover pompare
RPM/CE mentre il controllo è già nello stack. In quel caso si saltano ulteriori
step dello stesso controllo. Un thread di prep separato non può continuare a
pompare le medesime istanze quando il worker station è attivo.

`lx_platform` non contiene un context pointer: le callback devono riferirsi a
un singleton persistente. Deve restare vivo almeno quanto ath10k, insieme a
`WlfwBoardBinding`, `gw_snoc_board`, tabelle SMMU, store firmware e mappe fisiche.

## Contratti dei callback e verifiche hardware

Il runtime `guamp_xnu_sm6115_runtime.cpp` ora implementa questi collegamenti;
le proprieta elettriche, gli ACK reali e l'ownership restano da verificare
sul telefono. Il percorso diagnostic scan precede l'integrazione OS.

- `assignMsa`: istanziare `MsaAssignment` con `scm::Transport` reale e un payload
  fisico di almeno 512 byte, allineato a 64 byte, sotto 4 GiB e fuori dalla MSA.
  Assegnare esclusivamente le regioni ricevute da WLFW; ciascuna deve essere
  allineata a 4 KiB e contenuta in `0x51900000..0x51a00000`.
- `power`: il callback WLFW deve attendere i voti reali RPM. Il callback board
  ath10k può concedere/rilasciare l'accesso logico soltanto dopo conferma WLFW;
  `WifiPower` mantiene fisicamente i voti per questo boot.
- SMMU: allocare e mantenere descriptor/alias NC delle tabelle prima del
  `WlanSmmu::start()`. Non rilasciarle se l'invalidation fallisce.
- Modem: confermare reset/stop PAS sul percorso di errore prima di liberare
  memoria condivisa, mappe o DMA. Un fallimento XPC/CE o un timeout non prova
  che il remoto abbia smesso di usare tali risorse.
- Rete OS: il bridge necessita ancora delle API `redwood_net`/`redwood_sta` e
  dell'ABI Apple80211/IOKit effettivo del kernel target iOS 17. Una scansione
  radio headless può precedere questo attach, ma non prova la UI Wi-Fi di iOS.

## File osservati

- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampWifiPrep.inc`
- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampModem.inc`
- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampWifiControl.inc`
- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampWlfw.inc`
- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampMsa.inc`
- `../../sorgenti-wifi-modem/drivers/NativeEmmc/GuampSmmu.inc`
- `guamp_xnu_dma.h`, `guamp_xnu_station.c`, `guamp_wlfw_bridge.h`
- `../ath10k-port/gw_snoc_tail.inc`, `gw_snoc_api.c`, `nw_abi.h`
- `../ath10k-port/vendor/shim/src/lx_rt.c`
