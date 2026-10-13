;; Domaene der Missionsplanung (Layer 1). Drei Aktionen, siehe Projektstand:
;;   gehe_zu     - zu einem verbundenen Ort gehen (Kosten = Strecke in cm)
;;   balanciere  - am Ort sicher stehen/balancieren (BalancierAktion, Layer 3)
;;   warte       - am Ort eine feste Zeit warten
;; Ortsnamen und Aktionsnamen ASCII (Fast Downward ist bei Umlauten heikel).
(define (domain roboter-mission)
  (:requirements :strips :typing :action-costs)
  (:types ort)
  (:predicates
    (at ?o - ort)
    (verbunden ?a ?b - ort)
    (besucht ?o - ort)
    (balanciert ?o - ort)
    (gewartet ?o - ort))
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
    :effect (and (gewartet ?o) (increase (total-cost) 100))))
