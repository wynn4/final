%TF and bode example in matlab

s = tf('s');

G = 1/(s^2 + s + 1);

bode(G)
%step(G)
%impulse(G)