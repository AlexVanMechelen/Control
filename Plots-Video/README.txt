------------------
SSF_vs_ESSF_PI.png
------------------
De beginwaarde 0.1 rad werd opgelegd voor de hoek en 0 m voor de positie.
Als stelwaarde werd 1.2 m meegegeven. De implementatie van Extended State
Space Feedback voor de controle van de positie biedt duidelijk een meerwaarde
voor zowel de 'rise time' als voor de 'settling time'. Dit gaat echter ten
kostte van 'settling time' van de hoek. Uiteindelijk vormt dit geen al te
groot probleem aangezien de hoek ondanks deze vertraging nog steeds eerder
'gesettled' is dan de positie (voor eenzelfde foutmarge in de definitie van
'settle time').

----------------
ESSF_I_vs_PI.png
----------------
De beginwaarde 0.1 rad werd opgelegd voor de hoek en 0 m voor de positie.
Als stelwaarde werd 1.2 m meegegeven. Zoals verwacht is de Extended State
Space Feedback met PI net iets sneller dan deze met slechts een I.
Dit is aangezien de integrator slechts vanaf de tweede tijdstap kan reageren
op de aangelegde stap.


