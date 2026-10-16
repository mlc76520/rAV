# rAV — visualiseur et chaîne audio du « SalonRPI »

L'utilisateur échange en français. Le dépôt est public (README en anglais) :
le code, ses commentaires et **les textes affichés à l'écran sont en anglais**.
Garder le style et la densité de commentaires de `rav_unified.cpp`.

## Vue d'ensemble

- **Matériel** : Raspberry Pi 3B+ (Debian 13 trixie, arm64) avec :
  - deux OLED SSD1309 128×64 en SPI ;
  - quatre boutons (VIZ, PLAY, FW, RW) et un encodeur rotatif avec bouton ;
  - une sortie audio : HifiBerry Digi (S/PDIF) ou l'audio intégré.
- **Chaîne audio** : MPD et Bluetooth A2DP passent par **PipeWire** vers la sortie par défaut. PipeWire tourne en instance unique sous l'utilisateur `mpd`, démarrée au boot par le linger systemd.
- **Visualiseur** : il capte *passivement* le monitor de cette sortie via l'API PipeWire native. Si PipeWire est injoignable, il se replie sur ALSA (`hw:Loopback,1`).
- **Installation** : tout est mis en place par le paquet Debian **`rav-audio`** (`packaging/`).

## Fichiers

- `rav_unified.cpp` : tout le visualiseur.
  - couche matérielle GPIO/SPI (bcm2835 pour le Pi 3, lgpio pour le Pi 5) ;
  - `MPDClient`, `BluetoothMonitor` (D-Bus bluez, AVRCP) et `FontManager` (FreeType) ;
  - `AudioProcessor` (capture, tête de lecture virtuelle, FFTW) ;
  - les visualisations, l'éditeur de playlist, l'encodeur et `VisualizerApp`.
- `packaging/build-deb.sh` : compile et construit le `.deb`.
- `packaging/root/` : arborescence installée telle quelle. Tout ce qui est sous `etc/` devient un conffile.
- `packaging/debian/` : `control.in`, `postinst`, `prerm` et `postrm`.
- `trixel-square.ttf` : la police utilisée. Elle est cherchée dans le répertoire de travail du service, `/usr/share/rav-audio` ; `build-deb.sh` la prend à la racine du dépôt.
- `README.md` (public, en anglais), `img/`, `3D files/` (boîtier imprimé en STL/GLB) et `LICENSE`.
- `bluetooth_a2dp_setup.txt` : l'ancienne configuration bluez-alsa, gardée pour référence seulement.
- Si le README est concerné par une modification (dépendances, chaîne audio, comportement), le mettre à jour.

## Compiler, empaqueter, installer

- **Dépendances de build** : la liste `apt install` en tête de `rav_unified.cpp`, plus `libpipewire-0.3-dev`.
  - libbcm2835 ne vient pas de Debian : elle se compile depuis airspayce.com et est liée en statique.
- **Paquet** : `packaging/build-deb.sh <version>` produit `packaging/rav-audio_<version>_arm64.deb`. La compilation se fait sur le Pi et prend environ 30 s.
- **Matériel cible** : le build utilise `-DUSE_BCM2835`, donc il vise le Pi 3. Pour un Pi 5, il faut lgpio, sans `-DUSE_BCM2835` ; cette variante n'a pas été testée depuis le refactoring.
- **Installation** : `sudo apt install ./packaging/rav-audio_<v>_arm64.deb`, jamais `dpkg -i`, qui n'installe pas les dépendances. Incrémenter la version à chaque build installé.
- **Pas de copie manuelle du binaire** : le paquet gère le remplacement. En cas de copie manuelle, l'exécutable en cours d'exécution renvoie « Text file busy » ; il faut alors faire un `mv` par-dessus.

## Ce que le paquet met en place

- **Fichiers fournis** :
  - le binaire `/usr/bin/rav-visualizer` et l'unité `audio-visualizer.service` (root, à cause de `/dev/mem` pour bcm2835, avec `StateDirectory=rav-audio`) ;
  - les unités `bt-agent` et `bt-discoverable` ;
  - `/etc/rav-audio/mpd.conf`, choisi via `MPDCONF` dans un drop-in de `mpd.service`. `/etc/mpd.conf` n'est pas utilisé.
- **Configuration PipeWire et WirePlumber** :
  - `10-rav-audio.conf` : graphe à 44100 Hz ;
  - `50-rav-audio.conf` :
    - bluez sans surveillance de siège (seat-monitoring) ;
    - rôle `a2dp_sink` ;
    - volume de sortie par défaut à 1.0 ;
    - priorités : DAC I2S (`soc_sound`), puis jack intégré (`bcm2835_audio`), puis HDMI ;
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
  - simple avertissement sur `config.txt` : `dtparam=spi=on`, et `dtoverlay=hifiberry-…` ou `dtparam=audio=on`.

## Inspecter et déboguer

- **Graphe PipeWire** : `sudo -u mpd XDG_RUNTIME_DIR=/run/user/$(id -u mpd) wpctl status`. Même préfixe pour `pw-top` et `pw-dump`.
- **Journal du visualiseur** : `journalctl -u audio-visualizer -f`. On y trouve l'état de la capture, la taille des paquets (« Audio delivery ») et les commandes Bluetooth.
- **État de MPD** : `printf "status\nclose\n" | nc -q1 localhost 6600`.
- **Mesure de latence hors ligne** : extraire la classe `AudioProcessor` du source dans un programme de test, lui ajouter une méthode `inject()` qui remplit le tampon circulaire, et livrer un signal synthétique (grosse caisse, caisse claire, charleston espacés) au rythme réel, par paquets de la taille de la capture. Comparer ensuite deux versions sur le temps nécessaire pour atteindre 50 % du pic de chaque bande.

## Pièges connus : ne pas réintroduire

- **Ne pas revenir à la chaîne ALSA `multi`(HifiBerry + snd-aloop).** Avec le noyau 6.18.50, `timer_source=hw:sndrpihifiberry` provoque des `-EIO` et des blocages d'environ 0,5 s ; le rtio de MPD tournait alors en boucle et le son avait des ratés. Sans `timer_source`, les deux horloges dérivent. C'est la raison de la migration vers PipeWire.
- **Volume de sortie** : WirePlumber règle la sortie à 0.4 par défaut. Elle doit rester à 1.0, le volume se règle sur le flux MPD (le mixer MPD = volume du flux PipeWire).
- **Priorité temps réel** : il n'y a pas de rtkit. Elle passe par `LimitRTPRIO` dans le drop-in `user@<uid>` ; les threads `data-loop` doivent être en SCHED_FIFO.
- **Instance PipeWire unique** : sans `ConditionUser=mpd`, une session SSH ou VS Code lance une instance concurrente pour la sortie et le Bluetooth.
- **Capture passive** : la capture doit rester passive (`node.passive`), sinon la sortie ne se met plus en veille, et l'ampli non plus.
- **Cache FFTW** : `ProtectHome=read-only`, donc le cache doit aller dans `STATE_DIRECTORY`. Sinon, chaque démarrage recalcule les plans FFT, ce qui prend 6 à 21 s.
- **Bluetooth** : `BluetoothMonitor::isConnected()` signifie « flux A2DP actif », pas « téléphone connecté ». Les boutons routent donc sur la présence d'un `org.bluez.MediaPlayer1`, et dès qu'un téléphone est connecté, play, next et previous vont vers lui.
- **`bt-discoverable`** : il peut partir avant que bluetoothd ait exporté le contrôleur, d'où `Restart=on-failure`.
- **Scripts de maintenance** (`set -e`) : utiliser des `if`, pas `[ test ] && cmd` en fin de boucle ou de bloc.

## Choix de conception : latence et rendu

- **FFT** : basse à 4096 points, avec un gain de √2 pour garder l'étalonnage fait en 8192 ; médiums à 2048 ; aigus à 512. Toutes les fenêtres sont alignées sur les échantillons les plus récents.
- **Tête de lecture virtuelle** : latence = la moitié du plus gros paquet reçu + 15 ms, soit 27 ms avec un quantum PipeWire de 1024.
- **Lissage** : montée par `integral_factor` (environ 50 ms), descente par la « gravité » (`gravity_factor`). **L'utilisateur tient à la gravité.** Si un décalage son/image subsiste, la prochaine piste est d'accélérer la montée seulement.
- **Avance du monitor** : le monitor PipeWire précède les enceintes d'environ un tampon de sortie, ce qui compense une partie de la latence d'analyse.

## Version Rust (`rust/`) : SSD1309 et SSD1322 fusionnés

- **Origine** : un seul programme remplace `rav_unified.cpp` (SSD1309 128×64 mono, Pi 3) et le portage Rust de la variante SSD1322 240×128 en niveaux de gris (Pi 5), qui vit dans `RPI5_SSD1322_Vizualiser_*/`, non suivi par git.
- **État au 2026-10-16** :
  - le paquet installe encore le C++ ;
  - avant la fusion, la version SSD1309 seule avait été testée sur le matériel ;
  - la version fusionnée passe `cargo check` et le test de rendu, mais **sa compilation release n'a pas encore abouti sur le Pi 3**. Le dernier essai a échoué, probablement par manque de mémoire à l'édition de liens LTO ; il faut la refaire, avec `-j 1`, et retester sur le matériel.
- **Choix du profil, au démarrage** : la variable `RAV_DISPLAY`, sinon `display = ssd1309|ssd1322` dans `/etc/rav-audio/visualizer.conf`, sinon SSD1309.
- **Contenu d'un profil** (`src/profile.rs`) : écran, taille, vitesse SPI, broches, police et ses tailles, FFT basse, table de bandes, sens de la molette, overlay de volume, liste des visualisations et fonctions activées.
  - SSD1309 : FW en GPIO 9, FFT de 4096 et 64 bandes, police trixel en 8/10/14 px, transition et extinction activées, sans visualisations cachées.
  - SSD1322 : FW en GPIO 4, FFT de 8192 et 120 bandes, police FreePixel en 8/16/24 px, avec visualisations cachées.
- **Affichage** (`src/display/`) : un `Canvas` commun en niveaux 0 à 15, à la taille du profil, et deux pilotes.
  - `ssd1309.rs` allume un pixel à partir du niveau 8 et range l'image par pages, en écriture synchrone.
  - `ssd1322.rs` envoie 4 bits par pixel via un thread d'écriture.
- **Visualisations** (`src/viz/`) : une famille par écran, `ssd1309/` (les 6 de rAV, dessinées en niveau 15) et `ssd1322/` (les 14 du SSD1322, dont 2 cachées).
  - Commun : le trait `Viz`, `Beat`, `Rng`, et l'éditeur de playlist (`editor.rs`), dont seule la mise en page diffère (`Layout`).
  - Communs aussi : `ui.rs` (texte défilant, `Titles::small()` pour SSD1309 et `Titles::default()` pour SSD1322) et `transition.rs`.
- **Repris du portage SSD1322** : `mpd`, `bluetooth` (avec le correctif de déconnexion), `rotary`, `audio` (FFT appairée, synchro sur le retard de la carte son) et `font`.
- **Différences avec le C++ rAV** :
  - le matériel passe par `gpiocdev` et `spidev`, sans root ;
  - pas de repli ALSA ;
  - rustfft remplace FFTW ;
  - si le téléphone est la source active, la molette règle son volume ;
  - s'y ajoutent la transition Matrix et l'extinction par appui long sur PLAY.
- **Compilation** : `cd rust && cargo build --release -j 2`, avec le Rust 1.85 de Debian, plus `libclang-dev`.
  - Comptez environ 28 min la première fois sur un Pi 3, puis 7 à 10 min. Limiter à `-j 2`, sinon la mémoire ne suffit pas.
  - `.cargo/config.toml` choisit des dépendances compatibles avec Rust 1.85, par exemple zbus 5.13. Ne pas exiger de versions plus récentes dans `Cargo.toml`.
- **Test de rendu sans matériel** : `cargo test --release render -- --nocapture`. Il écrit chaque visualisation des deux profils en PNG dans `rust/target/render/`, avec un seuil appliqué pour le SSD1309. C'est le seul moyen de vérifier le SSD1322 sur ce Pi.
- **GPIO 9 (bouton FW, SSD1309)** : le noyau le réserve comme MISO de SPI0, et le bouton est alors désactivé avec un avertissement. Pour le libérer, remplacer `dtparam=spi=on` par `dtoverlay=spi0-2cs,no_miso` dans `config.txt`.
- **Tester sans le paquet** : `sudo systemctl stop audio-visualizer`, puis `sudo env PIPEWIRE_RUNTIME_DIR=/run/user/$(id -u mpd) RAV_DISPLAY=ssd1309 …/rust/target/release/rav-visualizer`, lancé depuis `/usr/share/rav-audio`. Relancer ensuite le service.
- **Tester sans toucher à la file MPD** : `sudo -u mpd XDG_RUNTIME_DIR=/run/user/$(id -u mpd) pw-play <fichier>`.

## Non vérifié ou à faire

- La désinstallation du paquet n'a jamais été testée.
- Le nom du nœud du jack intégré, `alsa_output.platform-bcm2835_audio.*`, n'a pas été vérifié sur du matériel.
- La variante Pi 5 (lgpio) n'a pas été testée.
- `music_directory` dans `/etc/rav-audio/mpd.conf` est propre au SalonRPI : il pointe vers `/media/icarus3/music`.
