%TP1 Transformée de Fourier discrète

% Exercice 1 : Analyse spectrale de la TFD
%{
x(t) = a1sin(2πf1t) + a2sin(2πf2t)
Avec f1 = 270Hz, f2 = 290Hz, a1 = a2 = 1 et fe = 1200Hz
Le nombre d’échantillons fréquentiels étant fixé à Lff t = 512.

Question 1 :
Donner les représentations temporelles et spectrales du signal x
Pour N (nombre d’échantillons temporels), N = 32 et N = 64
%}
a1=1;a2=1;f1=270;f2=290;
fe=1200;
N1=32;
N2=64;
Lfft=512;
t1=(0:N1-1)/fe;
t2=(0:N2-1)/fe;
f=(-Lfft/2:Lfft/2-1)*fe/Lfft;
subplot(2,2,1);
x1=(a1*sin(2*pi*f1*t1))+(a2*sin(2*pi*f2*t1));
plot(t1,x1);
title('Representation temprelle de x pour N=32');

subplot(2,2,2);
x2=(a1*sin(2*pi*f1*t2))+(a2*sin(2*pi*f2*t2));
plot(t2,x2);
title('Representation temporelle de x pour N=64');

subplot(2,2,3);
y1=fftshift(fft(x1,Lfft));
plot(f,abs(y1));
title('Representation spectrale de x pour N=32');

subplot(2,2,4);
y2=fftshift(fft(x2,Lfft));
plot(f,abs(y2));
title('Representation spectrale de x pour N=64');

%{
Question 2:
Maintenant pour N = 128, représenter le spectre d’amplitude de x pour
Lfft = 64 et Lfft = 512.
%}

f1=270;
f2=290;
a1=a2=1;
fe=1200;
N=128;
t1=(0:N-1)/fe;
x1=a1*sin(2*pi*t1*f1)+a2*sin(2*pi*t1*f2);
l1=64;
l2=512;
F1=(-l1/2:l1/2-1)*fe/l1;
F2=(-l2/2:l2/2-1)*fe/l2;

subplot(2,1,1);
y1=fftshift(fft(x1,l1));
stem(F1,abs(y1));
grid on;
title('spectre d amplitude pour l1=64');
xlabel('fréquence en HZ');
ylabel('amplitude');

subplot(2,1,2);
y2=fftshift(fft(x1,l2));
stem(F2,abs(y2));
grid on;
title('spectre d amplitude pour l2=512')
xlabel('fréquence en HZ');
ylabel('amplitude');

%{
Question 3 :
On se place maintenant dans le cas où f1 = 270Hz, f2 = 320Hz,
a1 = 1, a2 = 0.1, N = 64 et Lfft = 512.
Appliquer la fenêtre de Hamming (fonction hamming) au signal x et
représenter le spectre correspondant.
Comparer par rapport à la fenêtre rectangulaire .
%}
f22=320; a22=0.1; N11=64; L11=512;
t=(0:1/fe:(N11-1)/fe);
f=(0:1/fe:(L11-1)/fe);
x=a1*sin(2*pi*f1*t)+a22*sin(2*pi*f22*t);
h=hamming(N11);
y1=h'.*x;
y2=fftshift(fft(y1,L11));

figure
subplot(2,1,1);
plot(f,abs(y2));
title('representation avec fenetre de hamming');

subplot(2,1,2);
y3=fftshift(fft(x,L11));
plot(f,abs(y3));
title('representation avec fenetre rectangulaire');

% Exercice 2 : Repliement de spectre (Aliasing)
%{
Question 1:
s(t) = A.cos[θ(t)]
La fréquence instantannée est définie par :fi(t) =1/2π d[θ(t)]/dt
fi(t) = f0 + λt
λ = 1000 Hz.s−1 , fi(0) = 0, T = 2s, Fe = 8000 Hz.
%}

T = 2;Fe = 8000;lambda = 1000;f0 = 0;

% Temps et échantillonnage
t = 0:1/Fe:T;
fi = f0 + lambda * t;
theta = 2 * pi * cumtrapz(t, fi);
A = 1;
% Signal modulé en fréquence
s = A * cos(theta);
% Affichage du signal
figure;
plot(t, s);
title('Signal modulé en fréquence');
xlabel('Temps (s)');
ylabel('Amplitude');
grid on;


%question 2 Représenter son spectre d’amplitude.

% Calcul de la transformée de Fourier
L = length(s); % Longueur du signal
frequencies = (-Fe/2 : Fe/L : Fe/2 - Fe/L); % Fréquences correspondantes
Spectrum = fftshift(abs(fft(s, L)));

% Affichage du spectre d'amplitude
figure;
plot(frequencies, Spectrum);
title('Spectre d''amplitude du signal modulé en fréquence');
xlabel('Fréquence (Hz)');
ylabel('Amplitude');
grid on;

%Question 3 : Procéder à l’écoute du signal avec la commande soundsc(s, Fe).
soundsc(s, Fe);










