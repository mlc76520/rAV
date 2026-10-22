# rAV — façade physique pour MPD

L'utilisateur échange en français. Le dépôt est public (README en anglais) :
le code, ses commentaires et **les textes affichés à l'écran sont en anglais**.
Garder le style et la densité de commentaires du code existant.

**Aucune référence personnelle dans le dépôt** (noms des boîtiers, adresses,
serveur, chemins de la maison) : elles vont dans `CLAUDE.local.md`, non suivi
par git, qui décrit l'installation réelle (boîtiers, serveur, réglages locaux).

## Vue d'ensemble

- **Deux variantes matérielles, un seul programme** (Debian 13 trixie, arm64) :
  - Raspberry Pi 3 avec deux OLED **SSD1309 128×64** monochromes en SPI ;
  - Raspberry Pi 5 avec deux OLED **SSD1322 240×128** en niveaux de gris ;
  - sortie : un DAC I2S (HifiBerry…) ou l'audio intégré.
- **Façade commune** : quatre boutons (VIZ GPIO 27, PLAY 26, FW 4, RW 6), un encodeur rotatif (17 / 5, bouton 13) et une LED (16). Le bouton FW est en GPIO 4 ; voir les pièges.
- **Chaîne audio** : MPD et Bluetooth A2DP passent par **PipeWire** vers la sortie par défaut. PipeWire tourne en instance unique sous l'utilisateur `mpd`, démarrée au boot par le linger systemd.
- **Présentation publique** (README, paquet) depuis le 2026-10-17 : une façade physique pour MPD (lecteur réseau avec molette, boutons et deux écrans), le visualiseur étant un des points forts, pas le sujet principal.
- **Visualiseur** : la version Rust (`rust/`), qui capte *passivement* le monitor de cette sortie via l'API PipeWire native.
- **Installation** : tout est mis en place par le paquet Debian **`rav-audio`** (`packaging/`), en version 2.x depuis le 2026-10-16.
- **Testé sur les deux variantes** (voir `CLAUDE.local.md`) : sur Pi 3 + SSD1309, 7 % de CPU et 10 Mo de mémoire.

## Fichiers

- `rust/` : le visualiseur.
  - `src/profile.rs` : les deux profils d'écran (SSD1309, SSD1322) et leur chargement ;
  - `src/app.rs` et `main.rs` : démarrage, commandes, boucle principale, animation CRT, veille, extinction ;
  - `src/display/` : le `Canvas` commun en niveaux 0 à 15 et les pilotes `ssd1309.rs` et `ssd1322.rs` ;
  - `src/viz/` : le trait `Viz` et l'éditeur de playlist commun (`editor.rs`, une `Layout` par écran : deux menus : Library sur le bouton de la molette (Queue / Artists / Albums / Folders / Playlists / Update database) et Settings sur l'appui long d'1 s sur PLAY (Network / Bluetooth / Restart / Power off, ces deux derniers avec confirmation) ; pile d'écrans dont chaque ligne porte son action, toujours rouverte sur le menu demandé), puis les familles `ssd1309/` (les 6 de rAV) et `ssd1322/` (14, dont 2 cachées) ;
  - `src/audio.rs` et `bands.rs` : capture PipeWire, FFT et tables de bandes ;
  - `src/system.rs` : nom d'hôte, interfaces réseau (getifaddrs) et Wi-Fi (`nmcli`, facultatif) pour le menu Settings ;
  - `src/mpd.rs`, `bluetooth.rs` (zbus, y compris `Powered`/`Discoverable` de l'adaptateur), `font.rs` (FreeType), `ui.rs`, `transition.rs`, `hw.rs` (gpiocdev, spidev) et `rotary.rs` ;
  - `src/render_test.rs` : le test de rendu hors ligne.
- `packaging/build-deb.sh` : compile `rust/` et construit le `.deb`.
- `packaging/root/` : arborescence installée telle quelle. Tout ce qui est sous `etc/` devient un conffile.
- `packaging/debian/` : `control.in`, `postinst`, `prerm` et `postrm`.
- `trixel-square.ttf` : la police du SSD1309. Elle est cherchée dans le répertoire de travail du service, `/usr/share/rav-audio` ; `build-deb.sh` la prend à la racine du dépôt.
- `rav_unified.cpp`, l'ancienne version C++ (SSD1309 seulement), a été retirée le 2026-10-17 : elle reste dans l'historique git (jusqu'au commit `2fb656b`).
- `README.md` (public, en anglais), `img/`, `3D files/` (boîtier imprimé en STL/GLB) et `LICENSE`.
- `bluetooth_a2dp_setup.txt` : l'ancienne configuration bluez-alsa, gardée pour référence seulement.
- `RPI5_SSD1322_Vizualiser_*/` : non suivi par git. C'est l'origine de la version SSD1322 (C++ et Rust), fusionnée dans `rust/`. Il contient aussi la police FreePixel.
- Si le README est concerné par une modification (dépendances, chaîne audio, comportement), le mettre à jour.

## Profils d'écran

- **Choix au démarrage** : la variable `RAV_DISPLAY`, sinon `display = ssd1309|ssd1322` dans `/etc/rav-audio/visualizer.conf`, sinon SSD1309.
- **Contenu d'un profil** (`src/profile.rs`) : écran, taille, vitesse SPI, broches, police et ses tailles, FFT basse, table de bandes, sens de la molette, overlay de volume, liste des visualisations et fonctions activées.
  - **SSD1309** : 6,25 MHz, FFT de 4096 avec un gain de √2 pour garder l'étalonnage fait en 8192, 64 bandes, police trixel en 8/10/14 px, molette `+1`. La transition et le menu Settings sont activés, sans visualisations cachées.
  - **SSD1322** : 20 MHz, FFT de 8192, 120 bandes, police FreePixel en 8/16/24 px, molette `-1`, avec visualisations cachées (appui long sur VIZ).
- **Pilotes d'écran** : `ssd1309.rs` allume un pixel à partir du niveau 8 et range l'image par pages, en écriture synchrone. `ssd1322.rs` envoie 4 bits par pixel via un thread d'écriture.

## Compiler, empaqueter, installer

- **Dépendances de build** : `cargo rustc libclang-dev libpipewire-0.3-dev libfreetype-dev pkg-config`. Le Rust 1.85 de Debian 13 suffit.
  - `.cargo/config.toml` choisit des dépendances compatibles avec Rust 1.85, par exemple zbus 5.13. Ne pas exiger de versions plus récentes dans `Cargo.toml`.
- **Compilation** : `cd rust && cargo build --release`.
  - **Sur un Pi 3 (1 Go), utiliser `-j 1`** : avec `-j 2`, le noyau tue rustc par manque de mémoire pendant l'édition de liens LTO. `build-deb.sh` le fait tout seul sous 2 Go de mémoire.
  - Comptez environ 30 min la première fois sur un Pi 3, puis environ 11 min.
- **Paquet** : `packaging/build-deb.sh <version>` produit `packaging/rav-audio_<version>_arm64.deb`. Incrémenter la version à chaque build installé.
- **Installation** : `sudo apt install ./packaging/rav-audio_<v>_arm64.deb`, jamais `dpkg -i`, qui n'installe pas les dépendances. L'installation redémarre PipeWire, ce qui coupe le son quelques secondes.
- **Test de rendu sans matériel** : `cargo test --release render -- --nocapture`. Il écrit chaque visualisation des deux profils en PNG dans `rust/target/render/`, avec un seuil appliqué pour le SSD1309. C'est le seul moyen de vérifier le SSD1322 sans le boîtier Pi 5.

## Ce que le paquet met en place

- **Fichiers fournis** :
  - le binaire `/usr/bin/rav-visualizer` et l'unité `audio-visualizer.service`, qui tourne **sous l'utilisateur `mpd`** avec `SupplementaryGroups=gpio spi` et `ProtectHome=read-only` ;
  - `/etc/rav-audio/visualizer.conf` (type d'écran) et `/etc/rav-audio/mpd.conf`, ce dernier choisi via `MPDCONF` dans un drop-in de `mpd.service` (`/etc/mpd.conf` n'est pas utilisé). MPD y écoute sur toutes les interfaces (`bind_to_address "::"`) : les boîtiers sont pilotés par des clients MPD du réseau, ne pas revenir à `localhost`. **`music_directory` n'est pas dans le paquet** : il est dans `/etc/rav-audio/mpd_local.conf`, créé par `postinst` s'il manque (valeur Debian `/var/lib/mpd/music`), jamais écrasé, retiré à la purge ;
  - `/usr/share/polkit-1/rules.d/50-rav-audio.rules` : il autorise `mpd` à éteindre et à redémarrer la machine (menu Settings), et rien d'autre ;
  - les unités `bt-agent` et `bt-discoverable` ;
  - la police `trixel-square.ttf`. **FreePixel n'est pas fournie**, faute de licence vérifiée : sur le boîtier SSD1322, la copier à la main dans `/usr/local/share/rav-audio/`, sinon c'est DejaVu qui s'affiche.
- **Configuration PipeWire et WirePlumber** :
  - `10-rav-audio.conf` : graphe à 44100 Hz par défaut, qui suit la fréquence de la source (44,1 à 192 kHz) pour éviter le rééchantillonnage. Le visualiseur demande toujours 44100 Hz, et PipeWire convertit son flux de capture ;
  - `50-rav-audio.conf` :
    - bluez sans surveillance de siège (seat-monitoring) ;
    - rôle `a2dp_sink` ;
    - volume de sortie par défaut à 1.0 ;
    - priorités : DAC I2S (`platform-soc_sound` sur Pi 3, `platform-soc_107c000000_sound` sur Pi 5), puis jack intégré (`bcm2835_audio`), puis HDMI ;
    - snd-aloop désactivé.
- **Drop-ins `ConditionUser=mpd`** sur `pipewire.service`, `pipewire.socket` et `wireplumber.service`, dans `/usr/lib/systemd/user`.
- **Généré par `postinst`**, car cela dépend de l'UID de `mpd`, et retiré par `postrm` :
  - `/etc/systemd/system/{mpd,audio-visualizer,user@<uid>}.service.d/50-rav-audio-runtime.conf` ;
  - leur contenu : `XDG_RUNTIME_DIR` pour MPD, `PIPEWIRE_RUNTIME_DIR` pour le visualiseur, et `LimitRTPRIO` pour PipeWire.
- **Autres actions de `postinst`** :
  - `loginctl enable-linger mpd` ;
  - désactivation de bluealsa (réactivé à la désinstallation s'il était actif) ;
  - réglages bluez dans `main.conf` : `Class`, `AlwaysPairable`… ;
  - création de `/etc/bluetooth/pin.conf` aléatoire s'il manque. **Ne jamais mettre de PIN dans le paquet.**
  - mise à jour depuis une version 1.x (C++) sur un Pi 3 : arrêt de l'ancien visualiseur, puis remise des broches SPI dans le bon état (voir les pièges) ;
  - simples avertissements : `config.txt` (`dtparam=spi=on`, et `dtoverlay=hifiberry-…` ou `dtparam=audio=on`) et police FreePixel manquante avec le SSD1322.

## Inspecter et déboguer

- **Graphe PipeWire** : `sudo -u mpd XDG_RUNTIME_DIR=/run/user/$(id -u mpd) wpctl status`. Même préfixe pour `pw-top` et `pw-dump`.
- **Journal du visualiseur** : `journalctl -u audio-visualizer -f`. On y trouve le profil, l'état de la capture, les boutons indisponibles, les changements de visualisation et les événements Bluetooth.
- **État de MPD** : `printf "status\nclose\n" | nc -q1 localhost 6600`.
- **Tester un binaire sans le paquet** : `sudo systemctl stop audio-visualizer`, puis `sudo env PIPEWIRE_RUNTIME_DIR=/run/user/$(id -u mpd) RAV_DISPLAY=ssd1309 …/rust/target/release/rav-visualizer`, lancé depuis `/usr/share/rav-audio`. Relancer ensuite le service.
- **Jouer du son sans toucher à la file MPD** : `sudo -u mpd XDG_RUNTIME_DIR=/run/user/$(id -u mpd) pw-play <fichier>`.
- **Lire un bouton** : `sudo gpioget -c gpiochip0 -b pull-up <gpio>` (« active » = relâché). Ne marche que si le visualiseur ne tient pas la ligne.
- **Vérifier la règle polkit** : `sudo pkcheck --action-id org.freedesktop.login1.power-off --process $(pgrep -x rav-visualizer)`.

## Pièges connus : ne pas réintroduire

- **Ne pas revenir à la chaîne ALSA `multi`(HifiBerry + snd-aloop).** Avec le noyau 6.18.50, `timer_source=hw:sndrpihifiberry` provoque des `-EIO` et des blocages d'environ 0,5 s ; le rtio de MPD tournait alors en boucle et le son avait des ratés. Sans `timer_source`, les deux horloges dérivent. C'est la raison de la migration vers PipeWire.
- **Volume de sortie** : WirePlumber règle la sortie à 0.4 par défaut. Elle doit rester à 1.0, le volume se règle sur le flux MPD (le mixer MPD = volume du flux PipeWire).
- **Priorité temps réel** : il n'y a pas de rtkit. Elle passe par `LimitRTPRIO` dans le drop-in `user@<uid>` ; les threads `data-loop` doivent être en SCHED_FIFO.
- **Instance PipeWire unique** : sans `ConditionUser=mpd`, une session SSH ou VS Code lance une instance concurrente pour la sortie et le Bluetooth.
- **Capture passive** : la capture doit rester passive (`node.passive`), sinon la sortie ne se met plus en veille, et l'ampli non plus.
- **`ProtectHome=read-only`, pas `yes`** : `yes` masque aussi `/run/user/`, où se trouve le socket PipeWire de `mpd`, et la capture échoue en boucle.
- **Bouton FW en GPIO 4** : il était en GPIO 9, mais le noyau réserve cette broche comme MISO de SPI0. Les boîtiers ont été recâblés sur GPIO 4. Si une broche de bouton est prise, le programme désactive seulement ce bouton et l'écrit dans le journal.
- **Broches SPI après l'ancien C++** : à sa sortie, la bibliothèque bcm2835 remet GPIO 7 à 11 en entrée (`bcm2835_spi_end`). spidev n'affiche alors plus rien, alors que les boutons marchent toujours. Pour rétablir : `sudo pinctrl set 9-11 a0 && sudo pinctrl set 7-8 op dh`, ou redémarrer. `postinst` le fait lors d'une mise à jour depuis la 1.x.
- **Bluetooth** :
  - le transport A2DP actif (« streaming ») n'est pas la même chose qu'un téléphone connecté. Les boutons et la molette vont au téléphone quand il a un `org.bluez.MediaPlayer1` et qu'il est la source active : il diffuse, ou MPD ne joue pas.
  - à la déconnexion, bluez supprime le transport sans changer son état. Il faut écouter `InterfacesRemoved` et `Device1.Connected`, sinon l'écran reste sur le téléphone. C'était aussi un défaut de la version SSD1322 d'origine.
- **`bt-discoverable`** : il peut partir avant que bluetoothd ait exporté le contrôleur, d'où `Restart=on-failure`.
- **Scripts de maintenance** (`set -e`) : utiliser des `if`, pas `[ test ] && cmd` en fin de boucle ou de bloc.
- **Pi 3 : compiler avec `-j 1`** (voir plus haut).
- **Bibliothèque en NFS, lue par MPD lui-même** (`nfs://`, libnfs), sans montage dans le fstab. Si le nom du serveur se résout aussi en IPv6, mettre **son adresse IPv4** : un export limité au réseau IPv4 refuse sinon la connexion (`MNT3ERR_ACCES`). L'export doit avoir `insecure`, car MPD ne se connecte pas depuis un port privilégié. `playlist_directory` doit rester local (`/var/lib/mpd/playlists`) ; les radios partagées sont dans le dossier `playlists/` de la bibliothèque, et le menu Playlists les liste avec les playlists enregistrées.

## Choix de conception : latence et rendu

- **Synchro son/image** : le monitor PipeWire précède les enceintes d'environ un tampon de sortie, mesuré en direct dans `/proc/asound`. Les fenêtres FFT sont centrées sur l'échantillon qui sera audible quand l'image s'affichera, et se replient sur les échantillons les plus récents quand l'avance manque.
- **FFT** : une FFT complexe pour gauche et droite à la fois, en trois tailles (basse selon le profil, médiums 2048, aigus 512).
- **Lissage** : montée par `integral_factor`, descente par la « gravité » (`gravity_factor`). **L'utilisateur tient à la gravité.** Si un décalage son/image subsiste, la prochaine piste est d'accélérer la montée seulement.

## Historique

- **Avant le 2026-10-16** : le paquet 1.x installait `rav_unified.cpp`, qui tournait en root à cause de `/dev/mem` (bcm2835) et gardait un cache FFTW dans `StateDirectory`.
- **2026-10-16** :
  - fusion des deux portages Rust en un seul programme à profils ;
  - passage du paquet à la version Rust : 2.0.0, puis 2.0.1 (`ProtectHome`), puis 2.0.2 (FW en GPIO 4) ;
  - 2.0.3 : `visualizer.conf` enfin suivi par git (il était masqué par `visualizer.*` dans `.gitignore`), règle de priorité I2S valable sur Pi 5, MPD sur toutes les interfaces.
- **2026-10-17** : 2.0.4, menu de l'éditeur (Queue, Albums, Folders, Playlists).
- **2026-10-17** : 2.0.5, le graphe PipeWire suit la fréquence de la source au lieu de tout rééchantillonner à 44100 Hz, sans problème même sur une sortie S/PDIF.
- **2026-10-17** : 2.0.6, entrées Artists et Update database dans le menu de l'éditeur.
- **2026-10-17** : 2.0.7, overlay de volume du SSD1309 au style du SSD1322 (`ui::VolumeBar` commun) ; la transition Matrix recouvre l'ancienne visualisation et découvre la nouvelle en 750 ms.
- **2026-10-19** : 2.0.8, menu Settings sur l'appui long d'1 s sur PLAY (réseau, Bluetooth On/Off et visibilité, redémarrage et extinction avec confirmation), à la place de l'extinction directe par 3 s d'appui.
- **2026-10-19** : 2.0.9, bibliothèque lue en NFS par MPD au lieu d'un partage Samba monté en CIFS ; Playlists et Folders montrent les playlists du dossier `playlists/` de la bibliothèque.
- **2026-10-22** : 2.0.10, plus aucune référence personnelle dans le dépôt : `music_directory` passe dans `mpd_local.conf`, les réglages de la maison dans `CLAUDE.local.md`.
- **2026-10-22** : 2.0.11, Track Info d'un flux sans fin (radio) : barre de progression à hachures qui défilent, « mm:ss / LIVE » et icône de poste radio (`ui::progress_bar`, `ui::time_text`, `ui::radio_icon`, 16×10 sur SSD1309, 32×20 en niveaux de gris sur SSD1322).

## Non vérifié ou à faire

- La désinstallation du paquet n'a jamais été testée.
- `api.alsa.headroom` (avance pour la synchro) : à mesurer, puis décider s'il entre dans le paquet (voir `CLAUDE.local.md`).
- Le nom du nœud du jack intégré, `alsa_output.platform-bcm2835_audio.*`, n'a pas été vérifié sur du matériel.
- L'extinction et le redémarrage depuis le menu Settings n'ont pas été essayés en vrai : seules les règles polkit ont été vérifiées sous `mpd` (`pkcheck`, extinction et redémarrage).
- Le réglage Bluetooth du menu Settings (On/Off, Visible) dure jusqu'au redémarrage : `AutoEnable` et `bt-discoverable` le remettent à On et visible au boot.
