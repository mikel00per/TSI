(define (domain belkan)

  ; Includes!
  (:requirements :typing :fluents :strips)

  ; tipos en el domino
  (:types location zona orientacion - object jugador objeto personaje - location)

  ; predicados iniciales, siempre verdad
  (:predicates

    ; orientacion del jugador
    (orientado ?j - jugador ?o - orientacion)
    (sig-giro-izq ?o1 - orientacion ?o2 - orientacion)
    (sig-giro-der ?o1 - orientacion ?o2 - orientacion)

    ; caminos
    (conexion ?z1 - zona ?o - orientacion ?z2 - zona)

    ; reglas para las posiciones relativas a las entidades/objetos
    (posicion ?l - location ?z - zona)

    ; posesion de objetos en mano
    (mano ?j - jugador)
    (tengo-mano ?o - objeto)

    ; entrega de objetos
    (a-entregar ?o - objeto ?p - personaje)
  )


  ; --> Acciones que se pueden realizar

  ; acción de ir de una zona a otra
  (:action pasar
    :parameters (?j - jugador ?z1 ?z2 - zona ?o - orientacion)
    :precondition (and
      (posicion ?j ?z1)
      (orientado ?j ?o)
      (conexion ?z1 ?o ?z2)
    )
    :effect (and
      (not (posicion ?j ?z1))
      (posicion ?j ?z2)
    )
  )

  ; Acción de guirar izquierda
  (:action girar-izq
    :parameters (?j - jugador ?o ?o1 - orientacion)
    :precondition (and (orientado ?j ?o) (sig-giro-izq ?o ?o1))
    :effect (and
      (not (orientado ?j ?o))
      (orientado ?j ?o1)
    )
  )

  ; Acción de guirar izquierda
  (:action girar-der
    :parameters (?j - jugador ?o ?o1 - orientacion)
    :precondition (and
      (sig-giro-der ?o ?o1)
      (orientado ?j ?o)
    )
    :effect (and
      (not (orientado ?j ?o))
      (orientado ?j ?o1)
    )
  )

  ; accion de coger un objeto
  (:action coger-objeto
    :parameters (?j - jugador ?o - objeto ?z - zona)
    :precondition (and
      (posicion ?j ?z)
      (posicion ?o ?z)
      (not (mano ?j))
    )
    :effect (and
      (mano ?j)
      (tengo-mano ?o)
      (not (posicion ?o ?z))
    )
  )

  ; accion para dar un objeto
  (:action dar-objeto
    :parameters (?j - jugador ?o - objeto ?z - zona ?p - personaje)
    :precondition (and
      (tengo-mano ?o)
      (a-entregar ?o ?p)
      (posicion ?j ?z)
      (posicion ?p ?z)
    )
    :effect (and
      (not (mano ?j))
      (not (tengo-mano ?o))
      (not (a-entregar ?o ?p))
    )
  )

  ; accion de dejar un objeto en el suelo
  (:action dejar-objeto
    :parameters (?j - jugador ?o - objeto ?z - zona)
    :precondition (and
      (mano ?j)
      (tengo-mano ?o)
      (posicion ?j ?z)
    )
    :effect (and
      (not (mano ?j))
      (not (tengo-mano ?o))
      (posicion ?o ?z)
    )
  )

)
