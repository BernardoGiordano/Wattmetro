#include <DueTimer.h>

// A0 CORRENTE
// A1 TENSIONE

// supponiamo l'array di appoggio con una dimensione sufficiente 
// per memorizzare i valori delle letture su almeno due periodi
#define DIMENSIONE_MAX 100

// valori di fondo scala di arduino
#define VFS 3.3
#define NMAX 4095

// valori delle costanti k1 e k2
#define K2_TENSIONE 0.49212
#define K1_TENSIONE 0.43894
#define K1_CORRENTE 0.22002
#define K2_CORRENTE 0.4884

// variabili globali
int memoria_tensione[DIMENSIONE_MAX];
int memoria_corrente[DIMENSIONE_MAX];
int primoIndice = 0;
int secondoIndice = 0;
int nEventiDiTrigger = 0;

bool HALF_BUFFER = false;
bool FULL_BUFFER = false;
bool primoElemento = true;

// definisco delle variabili di appoggio che mi serviranno per memorizzare 
// in locale i valori e usarli in caso di evento di trigger rilevato
double appoggioPrecedenteTensione = 0;
double appoggioSuccessivoTensione = 0;
double appoggioPrecedenteCorrente = 0;
double appoggioSuccessivoCorrente = 0;

double numerico2tensione(int num) {
  return VFS*num/NMAX;
}

bool trigger(double precedente, double successivo) {
  return precedente*successivo < 0 && precedente <= 0;
}

// handler
void getVI() {
  int corrente = analogRead(A0);
  int tensione = analogRead(A1);
  if (primoElemento) { 
    appoggioSuccessivoTensione = tensione;
    appoggioSuccessivoCorrente = corrente;
    primoElemento = false;
  } else {
    // porto i vecchi new negli old
    appoggioPrecedenteTensione = appoggioSuccessivoTensione;
    appoggioPrecedenteCorrente = appoggioSuccessivoCorrente;
    // nei new metto i valori appena letti
    appoggioSuccessivoTensione = tensione;
    appoggioSuccessivoCorrente = corrente;

    bool eventoDiTrigger = trigger(numerico2tensione(appoggioPrecedenteTensione) - VFS/2, numerico2tensione(appoggioSuccessivoTensione) - VFS/2);

    if (eventoDiTrigger) {
      nEventiDiTrigger++;
    }

	// se non si sono ancora verificati eventi di trigger non fare nulla
    if (nEventiDiTrigger == 0) 
      return;

	// Se il numero di eventi di trigger è pari (e diverso da zero, per la condizione precedente), 
	// significa che abbiamo completato il riempimento della prima metà del buffer.
	// Se invece esso diventa dispari (ma non 1), abbiamo completato il 
	// riempimento della seconda metà del buffer.
    if (eventoDiTrigger) {
      if (nEventiDiTrigger % 2 == 0)
        HALF_BUFFER = true;
      else if (nEventiDiTrigger != 1)
        FULL_BUFFER = true;
    }
      
    if (!HALF_BUFFER && nEventiDiTrigger % 2 == 1) {
      memoria_tensione[primoIndice] = appoggioSuccessivoTensione;
      memoria_corrente[primoIndice] = appoggioSuccessivoCorrente;
      primoIndice++;
    } 
    
    else if (!FULL_BUFFER && nEventiDiTrigger % 2 == 0) { 
      int offset = DIMENSIONE_MAX/2;

      memoria_tensione[secondoIndice + offset] = appoggioSuccessivoTensione;
      memoria_corrente[secondoIndice + offset] = appoggioSuccessivoCorrente;
      secondoIndice++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  
  Timer3.attachInterrupt(getVI);
  Timer3.start(500);

  memset(memoria_tensione, 0, DIMENSIONE_MAX);
  memset(memoria_corrente, 0, DIMENSIONE_MAX);
}

double v_rms(bool isPrimaMeta, int n) {
  double tmp_rms = 0;

  for (int offset = isPrimaMeta ? 0 : 50, i = offset; i < n + offset; i++) {
    double valore = (numerico2tensione(memoria_tensione[i]) - K2_TENSIONE*VFS)/K1_TENSIONE; 
    tmp_rms += valore*valore;
  }

  tmp_rms = sqrt(tmp_rms/n);

  // restituisco il valore rms che abbiamo all'ingresso del trasduttore di tensione
  return tmp_rms * 100;
}

double i_rms(bool isPrimaMeta, int n) {
  double tmp_rms = 0;

  for (int offset = isPrimaMeta ? 0 : 50, i = offset; i < n + offset; i++) {
    double valore = (numerico2tensione(memoria_corrente[i]) - K2_CORRENTE*VFS)/K1_CORRENTE; 
    tmp_rms += valore*valore;
  }

  tmp_rms = sqrt(tmp_rms/n);

  // restituisco il valore rms che abbiamo all'ingresso del trasduttore di corrente
  return tmp_rms * 2;
}

double potenza_attiva(bool isPrimaMeta, int n) {
	double tmp_pot = 0;
	
	for (int offset = isPrimaMeta ? 0 : 50, i = offset; i < n + offset; i++) {
		double ik = (numerico2tensione(memoria_corrente[i]) - K2_CORRENTE*VFS)/K1_CORRENTE;
		double vk = (numerico2tensione(memoria_tensione[i]) - K2_TENSIONE*VFS)/K1_TENSIONE; 
		tmp_pot += vk*ik;
	}
	
	return tmp_pot / n;
}

void loop() {
  if (HALF_BUFFER) {
	double veff = v_rms(true, primoIndice);
	double ieff = i_rms(true, primoIndice);
	double potapp = veff*ieff;
	double potatt = potenza_attiva(true, primoIndice);
	double cosphi = potatt/potapp;
	
	char buffer[300];
	snprintf(buffer, 300, "Tensione RMS: %f V\nCorrente RMS: %f A\nPotenza attiva: %f W\nPotenza apparente: %f VA\nFattore di potenza: %f\n\n\n", veff, ieff, potatt, potapp, cosphi);

    primoIndice = 0;
    HALF_BUFFER = false;
  }

  else if (FULL_BUFFER) {
	double veff = v_rms(false, secondoIndice);
	double ieff = i_rms(false, secondoIndice);
	double potapp = veff*ieff;
	double potatt = potenza_attiva(false, secondoIndice);
	double cosphi = potatt/potapp;
	
	char buffer[300];
	snprintf(buffer, 300, "Tensione RMS: %f V\nCorrente RMS: %f A\nPotenza attiva: %f W\nPotenza apparente: %f VA\nFattore di potenza: %f\n\n\n", veff, ieff, potatt, potapp, cosphi);
	
    secondoIndice = 0;
    FULL_BUFFER = false;
  }
}