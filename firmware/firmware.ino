/**
 * Laboratorio di Misure e Diagnostica Industriale, A.A. 2016/2017
 * Modulo II: Progettazione e implementazione di un wattmetro numerico
 * 
 * Firmware per Arduino Due
 * 
 * Giordano Bernardo A13/854
 * Loffredo Tommaso A13/910
 * Egizio Ivan A13/916
 * Di Spazio Fabio A13/974
 */

#include <DueTimer.h>

// A0 CORRENTE
// A1 TENSIONE

// numero di periodi controllabile
#define NP 50

// frequenza di campionamento a 2kHz
#define FC 2000

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

int contatorePeriodi = 0;

// definisco variabili di appoggio globali che mi serviranno al momento della media su NP periodi
double appoggioFrequenza = 0;
double appoggioPotenzaAttiva = 0;
double appoggioTensioneRMS = 0;
double appoggioCorrenteRMS = 0;
double appoggioPotenzaApparente = 0;
double appoggioFattorePotenza = 0;

/**
 * @brief Funzione per convertire un valore numero in tensione
 * @param valore numerico da convertire
 * @return valore di tensione tra 0 e 3.3V
 */
double numerico2tensione(int num) {
  return VFS*num/NMAX;
}

/**
 * @brief Funzione per rilevare un evento di trigger
 * @param campione precedente (x-1)
 * @param valore successivo (x)
 * @return booleano
 */
bool trigger(double precedente, double successivo) {
  return precedente*successivo < 0 && precedente <= 0;
}

/**
 * Handler associato al timer
 */
void getVI() {
  // effettuo la lettura dei valori dagli ingressi analogici
  int corrente = analogRead(A0);
  int tensione = analogRead(A1);

  if (primoElemento) {
    // se sto rilevando il primissimo elemento dall'analogRead, metto direttamente il valore nell'appoggio successivo
    appoggioSuccessivoTensione = tensione;
    appoggioSuccessivoCorrente = corrente;
    primoElemento = false;
  } else {
    // ho già un valore relativo alla precedente chiamata dell'handler
    // porto i vecchi new negli old
    appoggioPrecedenteTensione = appoggioSuccessivoTensione;
    appoggioPrecedenteCorrente = appoggioSuccessivoCorrente;
    // nei new metto i valori appena letti
    appoggioSuccessivoTensione = tensione;
    appoggioSuccessivoCorrente = corrente;

    bool eventoDiTrigger = trigger(numerico2tensione(appoggioPrecedenteTensione) - VFS/2, numerico2tensione(appoggioSuccessivoTensione) - VFS/2);

    // aumento il contatore degli eventi di trigger in caso di evento rilevato
    if (eventoDiTrigger) {
      nEventiDiTrigger++;
    }

    // se non si sono ancora verificati eventi di trigger non faccio nulla e ritorno
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

  // pulisco i buffer di tensione e corrente in fase di inizializzazione
  memset(memoria_tensione, 0, DIMENSIONE_MAX);
  memset(memoria_corrente, 0, DIMENSIONE_MAX);
}

/**
 * @brief Calcolo del valore efficace di tensione
 * @param vero se il periodo del segnale è memorizzato nella prima metà del buffer, falso se nella seconda metà del buffer
 * @param numero di campioni nel periodo
 * @return valore efficace di tensione che comprende il prodotto per il fattore 100 introdotto dal trasduttore di tensione
 */
double v_rms(bool isPrimaMeta, int n) {
  double tmp_rms = 0;

  for (int offset = isPrimaMeta ? 0 : DIMENSIONE_MAX/2, i = offset; i < n + offset; i++) {
    double valore = (numerico2tensione(memoria_tensione[i]) - K2_TENSIONE*VFS)/K1_TENSIONE; 
    tmp_rms += valore*valore;
  }

  tmp_rms = sqrt(tmp_rms/n);

  // restituisco il valore rms che abbiamo all'ingresso del trasduttore di tensione
  return tmp_rms * 100;
}

/**
 * @brief Calcolo del valore efficace di corrente
 * @param vero se il periodo del segnale è memorizzato nella prima metà del buffer, falso se nella seconda metà del buffer
 * @param numero di campioni nel periodo
 * @return valore efficace di corrente che comprende il prodotto per il fattore 2 introdotto dal trasduttore di corrente
 */
double i_rms(bool isPrimaMeta, int n) {
  double tmp_rms = 0;

  for (int offset = isPrimaMeta ? 0 : DIMENSIONE_MAX/2, i = offset; i < n + offset; i++) {
    double valore = (numerico2tensione(memoria_corrente[i]) - K2_CORRENTE*VFS)/K1_CORRENTE; 
    tmp_rms += valore*valore;
  }

  tmp_rms = sqrt(tmp_rms/n);

  // restituisco il valore rms che abbiamo all'ingresso del trasduttore di corrente
  return tmp_rms * 2;
}

/**
 * @brief Calcolo della potenza attiva
 * @param vero se il periodo del segnale è memorizzato nella prima metà del buffer, falso se nella seconda metà del buffer
 * @param numero di campioni nel periodo
 * @return potenza attiva del segnale 
 */
double potenza_attiva(bool isPrimaMeta, int n) {
	double tmp_pot = 0;
	
	for (int offset = isPrimaMeta ? 0 : DIMENSIONE_MAX/2, i = offset; i < n + offset; i++) {
		double ik = (numerico2tensione(memoria_corrente[i]) - K2_CORRENTE*VFS)/K1_CORRENTE * 2;
		double vk = (numerico2tensione(memoria_tensione[i]) - K2_TENSIONE*VFS)/K1_TENSIONE * 100; 
		tmp_pot += vk*ik;
	}
	
	return tmp_pot / n;
}

/**
 * @brief stampa dei risultati su monitor seriale
 */
void stampa() {
  if (contatorePeriodi == NP) {
    Serial.print("\n\n\n\nTensione RMS: ");
    Serial.print(appoggioTensioneRMS / NP);
    Serial.println(" V");
    Serial.print("Corrente RMS: ");
    Serial.print(appoggioCorrenteRMS / NP);
    Serial.println(" A");
    Serial.print("Potenza apparente: ");
    Serial.print(appoggioPotenzaApparente / NP);
    Serial.println(" VA");
    Serial.print("Potenza attiva: ");
    Serial.print(appoggioPotenzaAttiva / NP);
    Serial.println(" W");
    Serial.print("Fattore di potenza: ");
    Serial.print(appoggioFattorePotenza / NP);
    Serial.print(" Gradi: ");
    Serial.println(acos(appoggioFattorePotenza / NP)*180/PI);
    Serial.print("Frequenza: ");
    Serial.print((FC / appoggioFrequenza) * NP);
    Serial.println(" Hz");
    Serial.print("Numero di punti totali: ");
    Serial.println(appoggioFrequenza);
    
    appoggioPotenzaAttiva = 0;
    appoggioPotenzaApparente = 0;
    appoggioFattorePotenza = 0;
    appoggioTensioneRMS = 0;   
    appoggioCorrenteRMS = 0;
    appoggioFrequenza = 0; 
    contatorePeriodi = 0;
  }  
}

/**
 * Ciclo della CPU. E' l'unico che ha la facoltà di settare a false le flag di HALF_BUFFER e FULL_BUFFER
 */
void loop() {
  if (HALF_BUFFER) {
    double veff = v_rms(true, primoIndice);
    double ieff = i_rms(true, primoIndice);
    double potapp = veff*ieff;
    double potatt = potenza_attiva(true, primoIndice);

    appoggioCorrenteRMS += ieff;
    appoggioTensioneRMS += veff;
    appoggioPotenzaAttiva += potatt;
    appoggioPotenzaApparente += potapp;
    appoggioFattorePotenza += potatt/potapp;
    appoggioFrequenza += primoIndice;

    contatorePeriodi++;
    stampa();

    primoIndice = 0;
    HALF_BUFFER = false;
  }

  else if (FULL_BUFFER) {
    double veff = v_rms(false, secondoIndice);
    double ieff = i_rms(false, secondoIndice);
    double potapp = veff*ieff;
    double potatt = potenza_attiva(false, secondoIndice);

    appoggioCorrenteRMS += ieff;
    appoggioTensioneRMS += veff;
    appoggioPotenzaAttiva += potatt;
    appoggioPotenzaApparente += potapp;
    appoggioFattorePotenza += potatt/potapp;
    appoggioFrequenza += secondoIndice;

    contatorePeriodi++;
    stampa();

    secondoIndice = 0;
    FULL_BUFFER = false;
  }
}
