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

    ; posesion de objetos
    (mano ?j - jugador)
    (tengo-mano ?o - objeto)
    (mochila ?j - jugador)
    (tengo-en-mochila ?o)

    ; tipos de terreno por zona
    (es-agua ?z - zona)
    (es-arena ?z - zona)
    (es-piedra ?z - zona)
    (es-bosque ?z - zona)
    (es-precipicio ?z - zona)

    ; necesidad de objeto para pasar por una zona determinada
    (necesito ?o - objeto ?z - zona)
  )

  ; Funciones
  (:functions
    (coste-t ?j - jugador)
    (coste ?z1 ?z2 - zona)
    (mochila ?j - jugador)
    (tam-maximo-mochila ?j - jugador)
    (beneficio ?j - jugador)
    (beneficio-parcial ?o - objeto ?p - personaje)
    (num-objetos-entregados ?j - jugador)
  )

  ; --> Acciones que se pueden realizar

  ; acción de ir de una zona a otra
  (:action pasar-normal
    :parameters (?j - jugador ?z1 ?z2 - zona ?o - orientacion)
    :precondition (and
      (posicion ?j ?z1)
      (orientado ?j ?o)
      (conexion ?z1 ?o ?z2)
      (not (es-precipicio ?z2))
      (not (es-agua ?z2))
      (not (es-bosque ?z2))
    )
    :effect (and
      (not (posicion ?j ?z1))
      (posicion ?j ?z2)
      (increase (coste-t ?j) (coste ?z1 ?z2))
    )
  )

  (:action pasar-agua
    :parameters (?j - jugador ?z1 ?z2 - zona ?o - orientacion ?obj - objeto)
    :precondition (and
      (posicion ?j ?z1)
      (orientado ?j ?o)
      (conexion ?z1 ?o ?z2)
      (es-agua ?z2)
      (necesito ?obj ?z2)
      (or (tengo-mano ?obj) (tengo-en-mochila ?obj))
    )
    :effect (and
      (not (posicion ?j ?z1))
      (posicion ?j ?z2)
      (increase (coste-t ?j) (coste ?z1 ?z2))
    )
  )

  (:action pasar-bosque
    :parameters (?j - jugador ?z1 ?z2 - zona ?o - orientacion ?obj - objeto)
    :precondition (and
      (posicion ?j ?z1)
      (orientado ?j ?o)
      (conexion ?z1 ?o ?z2)
      (es-bosque ?z2)
      (necesito ?obj ?z2)
      (or (tengo-mano ?obj) (tengo-en-mochila ?obj))
    )
    :effect (and
      (not (posicion ?j ?z1))
      (posicion ?j ?z2)
      (increase (coste-t ?j) (coste ?z1 ?z2))
    )
  )

  ; Acción de guirar izquierda
  (:action girar-izq
    :parameters (?j - jugador ?o ?o1 - orientacion)
    :precondition (and (orientado ?j ?o) (sig-giro-izq ?o ?o1))
    :effect (and
      (not (orientado ?j ?o))
      (orientado ?j ?o1)
      ; (increase (coste-t ?j) 1)
    )
  )

  ; Acción de guirar izquierda
  (:action girar-der
    :parameters (?j - jugador ?o ?o1 - orientacion)
    :precondition (and (sig-giro-der ?o ?o1) (orientado ?j ?o))
    :effect (and
      (not (orientado ?j ?o))
      (orientado ?j ?o1)
      ; (increase (coste-t ?j) 1)
    )
  )

  ; accion de coger un objeto
  (:action coger-objeto
    :parameters (?j - jugador ?o - objeto ?z - zona)
    :precondition (and
      (not (mano ?j))
      (posicion ?j ?z)
      (posicion ?o ?z)
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
      (posicion ?j ?z)
      (posicion ?p ?z)
    )
    :effect (and
      (not (mano ?j))
      (not (tengo-mano ?o))
      (increase (beneficio ?j) (beneficio-parcial ?o ?p))
      (increase (num-objetos-entregados ?j) 1)
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

  ; accion de guardar un objeto en mochila
  (:action guarda-mochila
    :parameters (?j - jugador ?o - objeto)
    :precondition (and
      (mano ?j)
      (tengo-mano ?o)
      (not (mochila ?j))
      (< (mochila ?j) (tam-maximo-mochila ?j))
    )
    :effect (and
      (not (mano ?j))
      (not (tengo-mano ?o))
      (tengo-en-mochila ?o)
      (increase (mochila ?j) 1)

    )
  )

  ; accion de sacar un objeto de la mochila

  (:action sacar-mochila
    :parameters (?j - jugador ?o - objeto)
    :precondition (and
      (not (mano ?j))
      (tengo-en-mochila ?o)
      (> (mochila ?j) 0)
    )
    :effect (and
      (mano ?j)
      (tengo-mano ?o)
      (not (tengo-en-mochila ?o))
      (decrease (mochila ?j) 1)
    )
  )
)
