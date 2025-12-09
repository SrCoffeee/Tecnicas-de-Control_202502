# Preguntas difíciles (y respuestas) — Control AS5600 + L298N

## 1) ¿Por qué el PID linealizado a 47° no sirve en 90°?
Porque \(k_\theta=MgL\cos\theta_0\). En \(90^\circ\), \(k_\theta=0\) y la planta es (casi) doble integrador amortiguado. Re-sintoniza allí, usa feed-forward gravitacional y/o un esquema robusto.

## 2) ¿Cómo garantizas estabilidad del SMC con fricción y saturación?
Con \(s=\dot e+\lambda e\) y \(u=u_{eq}-k\,\mathrm{sat}(s/\phi)\).
El candidato \(V=\tfrac12 s^2\) da \(\dot V\le -\eta|s|\) si \(k\) supera la cota de incertidumbres/\(K_u\).
La capa \(\phi\) reduce chatter manteniendo convergencia.

## 3) ¿Por qué fusionar AS5600 y MPU6050?
Encoder → mejor en posición; giroscopio → mejor en velocidad. Fusionarlos da \(\omega\) con menos ruido y mejor fase que derivar el encoder.

## 4) ¿Separar \(b\) de \(K_t^2/R\)?
Spin-down abierto: \(\tau_{open}=J/b\).
Cortocircuito: \(\tau_{short}=J/(b+K_t^2/R)\).
De ahí \(b\) y \(K_t^2/R\).

## 5) ¿Persistencia de excitación en RLS?
Requiere variación suficiente de \(\phi=[u,\ -\omega]\).
Perfiles tipo tren de impulsos/rampa lo garantizan; si no, el estimador no converge.

## 6) ¿Qué pasa si \(\hat K_u\) converge mal?
Proyección (acotar \(\hat K_u\)), regularizar \(P\), descartar muestras saturadas y umbralizar \(|\omega|\) para mitigar Coulomb.

## 7) ¿Pesos en \(H_\infty\)?
\(W_e\) fija BW y \(M_s\), \(W_u\) limita esfuerzo y \(W_t\) da roll-off.
Las condiciones \(\|W_e S\|_\infty<1\), \(\|W_u K\|_\infty<1\) guían el diseño.

## 8) Anti-windup por back-calculation:
Integra \(I \leftarrow I + (T_s/T_t)(u_{sat}-u_{raw})\) para “seguir” la salida saturada y evitar acumulación de error.

## 9) ¿Limitantes en 90° con 5 V?
El FF requerido puede acercarse a 1.0 de duty; queda poco margen.
Soluciones: mayor Vcc, contrapeso/resorte, o desplazar el punto de operación.

## 10) ¿Verificación de robustez?
Barre \(J,B\), añade saturación y zona muerta; evalúa \(M_p, T_s, e_{ss}, u\).
Un \(H_\infty\) bien ponderado o SMC con \(k\) suficiente mantiene desempeño aceptable.
