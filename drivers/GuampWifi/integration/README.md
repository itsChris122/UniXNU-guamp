# Provider Wi-Fi SM6115 / Guamp

Aggiornato il 28 ottobre 2026. È disponibile una prima integrazione sorgente che collega il loader modem al percorso radio fino a una scansione diagnostica. Compilazione dei singoli oggetti ARM64 e test dei protocolli sul PC superati; nessuna prova radio sul Moto e nessuna build completa XNU eseguita.

## Percorso implementato

`guamp_xnu_sm6115_provider.h` possiede le mappe SMEM, TCSR, APCS, RPM message RAM, Apps SMMU e WLAN ricavate dal DT Guamp. Gestisce lock SMEM locale + mutex hardware TCSR 3 e trasporto RPM con ACK. `GuampXnuBuffer` alloca memoria contigua e uncached per payload SCM e tabelle SMMU, verifica intervalli/maschere e trattiene i buffer pubblicati fino a stop remoto confermato.

`guamp_xnu_sm6115_runtime.cpp` compone questi owner con `WifiControl`, `MsaAssignment`, `WlanSmmu`, il DMA XNU e il bridge WLFW/ath10k. La sequenza è:

1. Il prep monta la sorgente firmware persistente e verifica il modem originale.
2. Il loader verifica PAS fermo e placement dell'immagine; il suo hook `prepare` collega RPM, attende i voti proxy e pubblica SMP2P/GLINK APPS prima dei primi caricamenti.
3. Dopo AUTH_AND_RESET accettato, l'hook `started` avvia il pump SMP2P → GLINK → QRTR → QFTP/SERVREG → QMI WLFW.
4. WLFW assegna le regioni MSA attraverso SCM. Il callback di potenza prepara SMMU/DMA prima dei rail WLAN e attende gli ACK RPM. Un servizio che dichiara WLAN già in esecuzione viene rifiutato prima di riconfigurare SMMU.
5. Dopo BDF, calibrazione dichiarata e indicazione FirmwareReady, ath10k riceve chip ID reale, registri, DMA e callback WLFW. Il worker richiede una scansione e registra completamento e BSS.

Il pump continua a servire CE anche nelle attese HTC/WMI interne al probe. `gw_snoc_poll_ce()` non richiama il dispatcher; il bootstrap e il worker station si passano l'ownership delle stesse istanze. Gli IRQ CE restano disabilitati perché il port usa polling e non ha handler XNU per quei 12 IRQ.

Correzione alla precedente diagnosi: WLFW raggiunge FirmwareReady autonomamente, dopo MSA/BDF/calibrazione e indicazione 0x21. `wlan_enable` effettua invece Configure → Mode → Enabled. Il provider completa prima quel bootstrap; il bridge attende lo stesso prerequisito.

## Inserimento nel package kernel

Non è stato modificato lo script di build del kernel attualmente installato. Per integrare il componente nella futura immagine:

- Compilare `guamp_xnu_sm6115_runtime.cpp` e `guamp_xnu_dma.cpp` con le stesse opzioni XNU del target, aggiungendo gli include di `integration` e dei sorgenti `NativeEmmc`. Nel solo archivio esportato, i sorgenti core sono in `checkpoint-sorgenti/NativeEmmc`.
- Compilare `NativeEmmc.cpp` con `GUAMP_WIFI_RUNTIME=1` e l'include di `integration`: `GuampWifiPrep.inc` collega i cinque entry point C del runtime tramite `ModemBootHooks`.
- Inserire anche i 25 oggetti del port ath10k e le loro dipendenze nel prelink effettivo. Il checker verifica gli oggetti e i simboli locali; il collegamento contro il kernel target completo rimane da eseguire.
- Avviare la prova diagnostica con `guamp-wifi-start=1`. Questo implica il prep e il cold boot modem verificato; non richiede un `guamp-modem-start=1` separato. Senza il componente compilato, il flag esegue il prep e registra che il runtime manca.

Il percorso è progettato per un solo cold boot: dopo un errore non riutilizza nello stesso boot regioni TrustZone o DMA di stato incerto. I voti fisici rimangono mantenuti per il boot. Il teardown libera le risorse solo dopo shutdown PAS confermato e dopo aver fermato DMA/SMMU; un errore dopo l'avvio della station trattiene tutto fino al riavvio.

## Log della prima prova

| Stage | Significato |
|---|---|
| 1–3 | mapping, canale RPM e voti proxy |
| 4 | endpoint APPS preparati prima del caricamento modem |
| 5 | PAS autenticato; negoziazione modem/WLFW in corso |
| 6 | firmware-ready, MSA/rail/SMMU/DMA disponibili |
| 7 | probe ath10k |
| 8 | scansione richiesta |
| 9 | scansione completata |
| 11 | errore: cercare il messaggio immediatamente precedente |

`GuampWiFi: SCAN COMPLETE: N BSS entries` è il completamento reale dell'evento scan. Zero BSS è un evento completato, ma non dimostra ricezione radio. Le righe BSS successive indicano BSSID, frequenza, RSSI e dimensione IE. Non sono stati osservati questi messaggi sul telefono: la prova hardware è ancora da fare.

## Interfaccia iOS e WPA2

`guamp_xnu_station.c` compila con gli header pubblici Redwood e con la generazione attuale Hakios (`hakios.net.h`/`hakios.sta.h`), usando le strutture dichiarate da quei progetti. Per il modo iOS servono `GUAMP_WIFI_REDWOOD=1`, il bridge station, e le implementazioni net/station/Apple80211/IOKit del kernel target. Il percorso aspetta `guamp_sm6115_wifi_network_ready()`: chiamare quel simbolo nel bootstrap BSD dopo `redwood_net_init()` o `hakios_net_init()`. Con `guamp-wifi-start=2` il worker attende il segnale prima di fare attach; il nostro package Guamp non contiene ancora tali componenti OS.

La station implementa WPA2-Personal PSK con CCMP e passa i test con access point simulato: PMK/PTK, MIC, replay, M4 ACK, GTK e rekey. Questo non certifica firmware, radio o ABI Apple80211 su iPadOS 17.7.11. Il metadata NON_BMI API5 è disponibile e il BDF è quello OEM Guamp; la compatibilità WMI/HTT con WLAN.HL 3.2.4-00782 resta da osservare sul dispositivo. WPA3, Enterprise, PMF e TKIP non sono implementati.

## Verifiche riproducibili

In WSL, dalla directory di lavoro:

```sh
python3 Codex/wifi/prepara-ath10k.py
python3 Codex/wifi/test-wpa2.py
python3 Codex/wifi/compile-provider-xnu.py --with-ios
python3 Codex/wifi/audit-provider-link.py
bash Codex/sorgenti-wifi-modem/drivers/NativeEmmc/tests/run-modem-tests.sh
bash Codex/sorgenti-wifi-modem/drivers/NativeEmmc/tests/run-transport-tests.sh
bash Codex/sorgenti-wifi-modem/drivers/NativeEmmc/tests/run-prep-runtime-check.sh
```

I percorsi XNU e degli header generati sono opzioni di `compile-provider-xnu.py`; i default corrispondono al PC corrente. Viene usata la dichiarazione reale `task_suspend_stats_t` tramite `PRIVATE=1`, senza sostituirne il tipo. I controlli compilano oggetti individuali e non creano un kernel. Risultati in `provider-check/provider-compile-report.json` e `provider-link-report.json`.

Riferimenti: [UniXNU Wi-Fi](https://unixnu.com/porting/network), [HackiOS](https://github.com/oosexgang/hackios-xnu-kernel), DT OEM in `../dtbo-analysis`, driver Qualcomm/Linux originali nei checkout locali. WCN7850/PCIe di sheng e WCN3990/SNOC di Guamp hanno provider differenti.
