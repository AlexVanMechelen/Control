--------------------------
Comparisons/PID_vs_SSF.png
--------------------------
State Space Feedback biedt een duidelijke verbetering qua 'rise time' en
'settling time' voor zowel de hoek- als positiecontrole.

------------------------------
Comparisons/SSF_vs_ESSF_PI.png
------------------------------
De beginwaarde 0.1 rad werd opgelegd voor de hoek en 0 m voor de positie.
Als stelwaarde werd 1.2 m meegegeven. De implementatie van Extended State
Space Feedback voor de controle van de positie biedt duidelijk een meerwaarde
voor zowel de 'rise time' als voor de 'settling time'. Dit gaat echter ten
kostte van 'settling time' van de hoek. Uiteindelijk vormt dit geen al te
groot probleem aangezien de hoek ondanks deze vertraging nog steeds eerder
'gesettled' is dan de positie (voor eenzelfde foutmarge in de definitie van
'settle time'). Een tweede meerwaarde van de ESSF is dat het werkt met een
feedback-lus voor de controle van de positie. De voorgaande methode bij SSF
van deling door de closed-loop gain is slechts een lineaire aanpak en dus
kwetsbaar voor modelleerfouten en niet redundant tegen perturbaties. De FB
lus van ESSF lost deze tekortkomingen op.

----------------------------
Comparisons/ESSF_I_vs_PI.png
----------------------------
De beginwaarde 0.1 rad werd opgelegd voor de hoek en 0 m voor de positie.
Als stelwaarde werd 1.2 m meegegeven. Zoals verwacht is de Extended State
Space Feedback met PI net iets sneller dan deze met slechts een I.
Dit is aangezien de integrator slechts vanaf de tweede tijdstap kan reageren
op de aangelegde stap.

----------
ESSF/*.png
----------
Alle figuren in deze directory zijn vergelijkingen tussen een simulatie in
Matlab via het lsim commando en een simulatie met de niet-lineaire
implementatie in Pyhton. Elke simulatie werd uitgevoerd met een initiële
perturbatie van 0.1 rad voor de hoek en 0 m voor de positie. In twee van de
simulaties was de stelwaarde voor de positie eveneens gelijk aan 0 m en in
de twee andere was deze gelijk aan 1 m, zodat de pendulum zich eveneens
moest verplaatsen. Welke figuur welke stelwaarde heeft is zichtbaar aan het
laatste cijfer in de bestandsnaam. Daarnaast werden twee simulaties uitgevoerd
met een PI Extended State Space Feedback en twee met slechts de I versie.
Er zal echter een miniem verschil zijn tussenbeide dat slechts in de lsim
zichtbaar is (zie 'Comparisons/ESSF_I_vs_PI.png').

Wat meteen opvalt bij elke figuur is het feit dat er in de Python simulatie
er ruis geïmplementeerd is op de meting, terwijl dit bij de lsim niet het
geval is. Bovendien is er de niet-lineariteit binnen de Python implementatie
die voor een iets tragere stabilisatie zorgt, zowel qua 'rise-time' als qua
'settling time'.

