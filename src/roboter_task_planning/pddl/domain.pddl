;; Domaene der Missionsplanung (Layer 1). Drei Aktionen, siehe Projektstand:
;;   gehe_zu     - zu einem verbundenen Ort gehen (Kosten = Strecke in cm)
;;   balanciere  - am Ort sicher stehen/balancieren (BalancierAktion, Layer 3)
;;   warte       - am Ort eine feste Zeit warten
;;   greife      - Gegenstand beidhaendig vom Tisch am Ort greifen (GreifAktion, Layer 3)
;;   ablegen     - gehaltenen Gegenstand am Ort ablegen
;; Ortsnamen und Aktionsnamen ASCII (Fast Downward ist bei Umlauten heikel).
(define (domain roboter-mission)
  (:requirements :strips :typing :action-costs)
  (:types ort gegenstand)
  (:predicates
    (at ?o - ort)
    (verbunden ?a ?b - ort)
    (besucht ?o - ort)
    (balanciert ?o - ort)
    (gewartet ?o - ort)
    (liegt_bei ?g - gegenstand ?o - ort)
    (haelt ?g - gegenstand)
    (haende_frei)
    (abgelegt ?g - gegenstand ?o - ort))
  (:functions
    (total-cost) - number
    (strecke ?a ?b - ort) - number)

  (:action gehe_zu
    :parameters (?von ?nach - ort)
    :precondition (and (at ?von) (verbunden ?von ?nach))
    :effect (and (not (at ?von)) (at ?nach) (besucht ?nach)
                 (increase (total-cost) (strecke ?von ?nach))))

  (:action balanciere
    :parameters (?o - ort)
    :precondition (at ?o)
    :effect (and (balanciert ?o) (increase (total-cost) 100)))

  (:action warte
    :parameters (?o - ort)
    :precondition (at ?o)
    :effect (and (gewartet ?o) (increase (total-cost) 100)))

  (:action greife
    :parameters (?g - gegenstand ?o - ort)
    :precondition (and (at ?o) (liegt_bei ?g ?o) (haende_frei))
    :effect (and (not (liegt_bei ?g ?o)) (not (haende_frei)) (haelt ?g)
                 (increase (total-cost) 100)))

  (:action ablegen
    :parameters (?g - gegenstand ?o - ort)
    :precondition (and (at ?o) (haelt ?g))
    :effect (and (not (haelt ?g)) (haende_frei) (liegt_bei ?g ?o) (abgelegt ?g ?o)
                 (increase (total-cost) 100))))
