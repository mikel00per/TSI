(define (problem ejercicio3)

  (:domain belkan)

  (:objects
    mkxv - jugador
    z0 z1 z2 z3 z4 z5 z6 z7 z8 z9 z10 z11 z12 z13 z14 z15 z16 z17 z18 z19 z20 z21 z22 z23 z24 - zona
    norte sur este oeste - orientacion
    Bruja Leonardo Principe Princesa Profesor - personaje
    oro rosa oscar manzana algoritmo bikini zapatillas - objeto
    precipicio agua bosque arena piedra - terreno
  )
  (:init
    (= (coste-t mkxv) 0)
    (orientado mkxv norte)
    (posicion mkxv z0)

    (sig-giro-der este sur)
    (sig-giro-der sur oeste)
    (sig-giro-der norte este)
    (sig-giro-der oeste norte)

    (sig-giro-izq sur este)
    (sig-giro-izq oeste sur)
    (sig-giro-izq este norte)
    (sig-giro-izq norte oeste)

    (a-entregar algoritmo Profesor)
    (a-entregar manzana bruja)
    (a-entregar oro Principe)
    (a-entregar oscar Leonardo)
    (a-entregar rosa Princesa)

    ; conexiones norte
    (conexion z20 norte z21) (conexion z17 norte z10)
    (conexion z10 norte z11) (conexion z11 norte z12)
    (conexion z12 norte z24) (conexion z22 norte z23)
    (conexion z9 norte z3)   (conexion z3 norte z0)
    (conexion z0 norte z1)   (conexion z1 norte z5)
    (conexion z17 norte z16) (conexion z18 norte z8)
    (conexion z8 norte z7)   (conexion z7 norte z6)
    (conexion z6 norte z13)  (conexion z15 norte z14)

    ; conexiones sur
    (conexion z21 sur z20) (conexion z10 sur z17)
    (conexion z11 sur z10) (conexion z12 sur z11)
    (conexion z24 sur z12) (conexion z23 sur z22)
    (conexion z3 sur z9)   (conexion z0 sur z3)
    (conexion z1 sur z0)   (conexion z5 sur z1)
    (conexion z16 sur z17) (conexion z8 sur z18)
    (conexion z7 sur z8)   (conexion z6 sur z7)
    (conexion z13 sur z6)  (conexion z14 sur z15)

    ; Conexiones este
    (conexion z23 este z24) (conexion z22 este z12)
    (conexion z12 este z5)  (conexion z5 este z6)
    (conexion z6 este z15)  (conexion z13 este z14)
    (conexion z11 este z4)  (conexion z4 este z0)
    (conexion z0 este z2)   (conexion z2 este z7)
    (conexion z21 este z10) (conexion z20 este z17)
    (conexion z10 este z9)  (conexion z9 este z8)
    (conexion z8 este z16)  (conexion z18 este z17)

    ; Conexiones oeste, inversas a las este
    (conexion z24 oeste z23) (conexion z12 oeste z22)
    (conexion z5 oeste z12)  (conexion z6 oeste z5)
    (conexion z15 oeste z6)  (conexion z14 oeste z13)
    (conexion z4 oeste z11)  (conexion z0 oeste z4)
    (conexion z2 oeste z0)   (conexion z7 oeste z2)
    (conexion z10 oeste z21) (conexion z17 oeste z20)
    (conexion z9 oeste z10)  (conexion z8 oeste z9)
    (conexion z16 oeste z8)  (conexion z17 oeste z18)

    (posicion oro z1)
    (posicion rosa z2)
    (posicion oscar z3)
    (posicion algoritmo z0)

    (posicion Principe z6)
    (posicion Princesa z8)
    (posicion Leonardo z10)
    (posicion Profesor z0)

    (posicion zapatillas z1)
    (posicion bikini z2)

    (= (coste z20 z21) 2)  (= (coste z17 z10) 2)
    (= (coste z10 z11) 4)  (= (coste z11 z12) 2)
    (= (coste z12 z24) 2)  (= (coste z22 z23) 2)
    (= (coste z9 z3) 4)    (= (coste z3 z0) 2)
    (= (coste z0 z1) 4)    (= (coste z1 z5) 2)
    (= (coste z17 z16) 4)  (= (coste z18 z8) 4)
    (= (coste z8 z7) 2)    (= (coste z7 z6) 2)
    (= (coste z6 z13) 2)   (= (coste z15 z14) 2)

    ; costos sur
    (= (coste z21 z20) 2)   (= (coste z10 z17) 2)
    (= (coste z11 z10) 4)   (= (coste z12 z11) 2)
    (= (coste z24 z12) 2)   (= (coste z23 z22) 2)
    (= (coste z3 z9) 4)     (= (coste z0 z3) 2)
    (= (coste z1 z0) 4)     (= (coste z5 z1) 2)
    (= (coste z16 z17) 4)   (= (coste z8 z18) 4)
    (= (coste z7 z8) 2)     (= (coste z6 z7) 2)
    (= (coste z13 z6) 2)    (= (coste z14 z15) 2)

    ; costos este
    (= (coste z23 z24) 5)   (= (coste z22 z12) 3)
    (= (coste z12 z5) 5)    (= (coste z5 z6) 3)
    (= (coste z6 z15) 5)    (= (coste z13 z14) 3)
    (= (coste z11 z4) 5)    (= (coste z4 z0) 4)
    (= (coste z0 z2) 5)     (= (coste z2 z7) 3)
    (= (coste z21 z10) 5)   (= (coste z20 z17) 4)
    (= (coste z10 z9) 4)    (= (coste z9 z8) 4)
    (= (coste z8 z16) 4)    (= (coste z18 z17) 4)

    ; costos oestes
    (= (coste z24 z23) 5)   (= (coste z12 z22) 3)
    (= (coste z5 z12) 5)    (= (coste z6 z5) 3)
    (= (coste z15 z6) 5)    (= (coste z14 z13) 3)
    (= (coste z4 z11) 5)    (= (coste z0 z4) 4)
    (= (coste z2 z0) 5)     (= (coste z7 z2) 3)
    (= (coste z10 z21) 5)   (= (coste z17 z20) 4)
    (= (coste z9 z10) 4)    (= (coste z8 z9) 4)
    (= (coste z16 z8) 4)    (= (coste z17 z18) 4)

    ; tipos de terreno
    (es-arena z0)         (es-arena z1)
    (es-arena z2)         (es-arena z3)
    (es-arena z4)         (es-agua z5)
    (es-arena z6)         (es-bosque z7)
    (es-arena z8)         (es-bosque z9)
    (es-arena z10)        (es-agua z11)
    (es-arena z12)        (es-precipicio z13)
    (es-precipicio z14)   (es-arena z15)
    (es-arena z16)        (es-arena z17)
    (es-precipicio z18)   (es-piedra z19)
    (es-piedra z20)       (es-piedra z21)
    (es-piedra z22)       (es-piedra z23)
    (es-precipicio z24)

    (necesito bikini z5)
    (necesito bikini z11)
    (necesito zapatillas z7)
    (necesito zapatillas z9)

  )

  (:goal
    (and
      (not (a-entregar algoritmo Profesor))
      (not (a-entregar oro Principe))
      (not (a-entregar oscar Leonardo))
      (not (a-entregar rosa Princesa))
    )
  )

  (:metric minimize (coste-t mkxv))

)
