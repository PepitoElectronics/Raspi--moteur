//============================================================================
// Name        : Moteur.cpp
// Author      : Oli - Juan
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <iostream>
#include "unistd.h"
#include <pthread.h>
#include <time.h>

#include "wiringPi.h"
#include <math.h>

#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <semaphore.h>

#include <signal.h> // gestion des signaux (pour nous : fermeture du programme)


//TCP
#include <arpa/inet.h>
#include <errno.h>


#define TIMER_RUN_TIME_S (5*60)  // temps approx d'exécution du programme, en secondes
#define TIMER_PERIOD_US 500  // période du timer, en microsecondes
//#define TIMER_PERIOD_US 4201  // période du timer, en microsecondes
#define OS_MAX_LATENCY_US 130  // latence max de l'OS temps-réel (cyclictest), en microsecondes

// En moyenne 2 microsecondes, mais pointe mesurée à 130microsecondes...
// probablement arrivée dans un cas où le kernel lui-même était temporairement sur-chargé
#define CLOCK_GETTIME_AVG_DURATION_NS (2100L)

#define USE_COMPENSATED_SLEEP 1  // compensation de latence, ou sleep naïf
/* Comparaison des 2 pour un rpi 3 A+ kernel 4.19.71-rt24-v7+; compilation release -03, prio FIFO 99
 * mesures à l'oscilloscope pendant 5 minutes, pour un timer à 2kHz (500us), mesures en us
 *
 *                          avg       min      max
 * sans compensation:       526       481      708
 * avec compensation:       500       479      638
 */

#define nBitsAEnvoyer 20 //Taille max du buffer d'envoi
#define gain 6
//TCP
#define PORT 1883
#define BUFFER_SIZE 32
#define PERIODE_CORRECTEUR 4000 // us
#define PERIODE_ECHEC 5000000 // us



int consigne = 0; // consigne initiale hard codée
float DiviseurKp; //Facteur de division du kp
float Kp;
float FacteurOubli=0.9985;//FacteurOubli=0.9985;

// Ce programme peut encore être amélioré, par exemple :
//   - en gérant les erreurs de temps-réel (à un certain % près)
//   - en vérifiant les valeurs d'erreur de nanosleep (voir 'man nanosleep')
//   - en gérant les très longs délais (actuellement: pas prévu pour délais ~ 1 seconde)


// Variables globales / defines, pas propre... à éviter au maximum.
// Mais plus facile ici, pour ne pas complexifier ce code d'exemple

const int clockPin = 0;  // numérotation wiringPi. On aura un signal d'horloge:
// chaque front montant ou descendant correspond à un tick du timer
const int dbgPin = 1;  // numérotation wiringPi


// CODE AJOUTEE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DEBUT
const int lgcRelPin = 11;
const int pwrRelPin = 31;
const int RXPin = 13;
const int TXPin = 14;
const int pwmPin = 26;

int INC=0;
pthread_mutex_t mutexINC;
int ratioTemps=8;

int messageRX[nBitsAEnvoyer];

const int sr1Pin = 6;
const int sr2Pin=10;
const int pinMasterRequest=15;

int angle=0;
int anglePrecedent=0;
int angleMax=65535;
int ecart;
int ecartPrecedent;
int ecartDixPrecedent;
int angleConsigne=0;


int server_socket, client_socket;


pthread_mutex_t mutexAngle;
pthread_mutex_t mutexAngleConsigne;

sem_t* semDemarrage;
sem_t* semDemarrageCom;
// CODE AJOUTEE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! FIN

// Fonctions du timer lui-même
void* rtSoftTimerThread(void* arg);
void* rtSoftTimerThreadCorrecteur(void* arg);
void* clockTimer(void* arg);
void* comTCP(void* arg);
void* TX(void* arg);
void* Correcteur(void *arg);
void* ChangeStatePin(void* arg);

void naiveSleepUntil(struct timespec* sleepEndTime);
void compensatedSleepUntil(struct timespec* sleepEndTime);

void initRelais();
void initSR(int sens);
void initRX();
void initTX();
void finRelais(int choix);
void PWM(int rc);
void TXnaif();
int exp2(int exponent);
int exptt(int base, int exponent);

int utime=600000;

// Prototypes

void generalSignalHandler(int);
void terminerProgramme();
// Création et initialisation de la structure POSIX
void generalSignalHandler(int signal)
{
    if (signal == SIGTERM)
    {
        std::cout << std::endl << "Exécution de terminerProgramme() en cours..." << std::endl;
        terminerProgramme();
        std::cout << "Fin." << std::endl;
        exit(0);
    }
    else
    std::cerr << "Signal non-géré" << std::endl;
}

// Activation des destructions__________________________________________________
void terminerProgramme()
{
    free(semDemarrage);
    close(client_socket);
    std::cout << "terminerProgramme." << std::endl;
}


int main() {

    /*************** GESTION SIGTERM ************************/
    struct sigaction signalAction;
    sigemptyset(&signalAction.sa_mask);
    signalAction.sa_flags = 0;
    // Fonction appelée lors de la signalisation
    signalAction.sa_handler = &generalSignalHandler;
    // Interception de SIGTERM uniquement
    if (sigaction(SIGTERM, &signalAction, NULL) == -1) // si erreur
    std::cerr << "Impossible d'intercepter SIGTERM !" << std::endl;

    /***********************************************************/


    // Allocation dynamique de la mémoire pour le sémaphore
    semDemarrage = (sem_t*)malloc(sizeof(sem_t));
    semDemarrageCom = (sem_t*)malloc(sizeof(sem_t));
    // Initialisation du sémaphore avec la valeur initiale 1
    sem_init(semDemarrage, 0, 0);
    sem_init(semDemarrageCom, 0, 0);


    // Initialiser WiringPi
    wiringPiSetup();
    pinMode(pwmPin, PWM_OUTPUT);

    std::cout << "Real-Time software Timer on Raspberry Pi 3" << std::endl;
    char hostname[100];
    gethostname(hostname, 100);
    std::cout << "Machine name: " << hostname << std::endl;

    // TID
    pthread_t rtThreadTid;
    pthread_t clockThreadTid;
    pthread_t correcteurThreadTid;
    pthread_t TCPTid;
    pthread_t TXTid;
    // Configuration et lancement du thread
    pthread_attr_t threadAttributes;
    pthread_attr_init(&threadAttributes);
    pthread_attr_setdetachstate(&threadAttributes, PTHREAD_CREATE_JOINABLE);
    pthread_create(&rtThreadTid, &threadAttributes, &rtSoftTimerThread, 0);
    pthread_attr_destroy(&threadAttributes);
    ////////////////////////////////////////
    pthread_attr_t threadClock;
    pthread_attr_init(&threadClock);
    pthread_attr_setdetachstate(&threadClock, PTHREAD_CREATE_JOINABLE);
    pthread_create(&clockThreadTid, &threadClock, &clockTimer, 0);
    pthread_attr_destroy(&threadClock);
    ////////////////////////////////////////
    ////////////////////////////////////////
    pthread_attr_t threadCorrecteur;
    pthread_attr_init(&threadCorrecteur);
    pthread_attr_setdetachstate(&threadCorrecteur, PTHREAD_CREATE_JOINABLE);
    pthread_create(&correcteurThreadTid, &threadCorrecteur, &rtSoftTimerThreadCorrecteur, 0);
    //pthread_create(&correcteurThreadTid, &threadCorrecteur, &Correcteur, 0);
    pthread_attr_destroy(&threadCorrecteur);
    ////////////////////////////////////////
    ////////////////////////////////////////

    pthread_attr_t threadComTCP;
    pthread_attr_init(&threadComTCP);
    pthread_attr_setdetachstate(&threadComTCP, PTHREAD_CREATE_JOINABLE);
    pthread_create(&TCPTid, &threadComTCP, &comTCP, 0);
    pthread_attr_destroy(&threadComTCP);
    ////////////////////////////////////////
    ////////////////////////////////////////

    pthread_attr_t threadTX;
    pthread_attr_init(&threadTX);
    pthread_attr_setdetachstate(&threadTX, PTHREAD_CREATE_JOINABLE);
    /*struct sched_param paramstx;
    paramstx.sched_priority = 50;
    pthread_setschedparam(TXTid, SCHED_FIFO, &paramstx);*/
    pthread_create(&TXTid, &threadTX, &TX, 0);
    pthread_attr_destroy(&threadTX);
    ////////////////////////////////////////
    initRX();
    initTX();


    pthread_t thread_MasterRequest;
    pthread_create(&thread_MasterRequest, NULL, &ChangeStatePin, NULL);

    // Fonction ajoutée
    //initSR(0);
    PWM(0);
    initRelais();
    //sleep(1);
    //finRelais(0);
    sem_post(semDemarrage);
    sem_post(semDemarrageCom);
    while(1){
        usleep(60);
        //TXnaif();
    }
    //finRelais();
    // Attente la fin du threads - join
    pthread_join(rtThreadTid, 0);
    return 0;
}


void initSR(int sens){
    if (sens==0) {
        pinMode(sr1Pin, OUTPUT);
        digitalWrite(sr1Pin, HIGH);
        pinMode(sr2Pin, OUTPUT);
        digitalWrite(sr2Pin, LOW);
    } else {
        pinMode(sr2Pin, OUTPUT);
        digitalWrite(sr2Pin, HIGH);
        pinMode(sr1Pin, OUTPUT);
        digitalWrite(sr1Pin, LOW);
    }
}
void PWM(int rc){

/*
    // Définir la fréquence PWM
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);  // Fréquence de base de 19.2 MHz divisée par 384 donne 50 kHz
    pwmSetRange(1024); // Plage de valeurs pour le rapport cyclique (de 0 à 1023)
*/
    int rapport_cyclique = 1024; // 50% de rapport cyclique par défaut

    // Définir le rapport cyclique initial
    pwmWrite(pwmPin, rc);

}
void initRelais(){
    pinMode(lgcRelPin, OUTPUT);
    pinMode(pwrRelPin, OUTPUT);
    sleep(1);
    digitalWrite(lgcRelPin, HIGH);
    sleep(1);
    digitalWrite(pwrRelPin, HIGH);

    std::cout << "Relais OK "<< std::endl;

}
void initRX(){
    pinMode(RXPin, INPUT);
}
void initTX(){
    pinMode(TXPin, OUTPUT);
    digitalWrite(TXPin, HIGH);
}

/*
 void TX(){

digitalWrite(TXPin, LOW);
int stateRX=1;
while(stateRX!=0){
stateRX=digitalRead(RXPin);18
std::cout << "Attend RX" << std::endl;
usleep(1);

}
int nBitsEnvoye=0;
while(nBitsEnvoye<nBitsAEnvoyer){
//std::cout << "ID boucle : " << nBitsEnvoye << std::endl;
pthread_mutex_lock(&mutexINC);
if(INC==ratioTemps/2){
digitalWrite(TXPin, HIGH);
std::cout << "Front montant" << std::endl;
} else if(INC==ratioTemps){
digitalWrite(TXPin, LOW);
messageRX[nBitsEnvoye]=digitalRead(RXPin);
nBitsEnvoye+=1;
std::cout << "Front descendant" << std::endl;
}
pthread_mutex_unlock(&mutexINC);
}
std::cout << "Fin message" << std::endl;
digitalWrite(TXPin, HIGH);
}
 * */
int exp2(int exponent){
    int base = 2;
    int result_multiply = 1;

    for (int i = 0; i < exponent; ++i) {
        result_multiply *= base;
    }
    return result_multiply;
}
int exptt(int base, int exponent){
    int result_multiply = 1;
    for (int i = 0; i < exponent; ++i) {
        result_multiply *= base;
    }
    return result_multiply;
}

void TXnaif(){
    bool pinHigh = false;
    digitalWrite(TXPin, LOW);
    //int local_consigne = consigne;
    int stateRX=1;
    while(stateRX!=0){
        stateRX=digitalRead(RXPin);
        //std::cout << "Attend RX" << std::endl;
        usleep(1);
    }
    int nBitsEnvoye=0;
    while(nBitsEnvoye<nBitsAEnvoyer){
        usleep(50);
        digitalWrite(TXPin, HIGH);
        //std::cout << "Front montant" << std::endl;
        usleep(25);
        messageRX[nBitsEnvoye]=digitalRead(RXPin);
        nBitsEnvoye+=1;
        usleep(25);
        digitalWrite(TXPin, LOW);
        //std::cout << "Front descendant" << std::endl;
    }
    //std::cout << "Fin message" << std::endl;
    digitalWrite(TXPin, HIGH);

    int resultat=0;
    for (int i=0;i<16;i++){
        //std::cout << "messageRX case n° "<< i<< " : " << messageRX[i] <<std::endl;
        resultat=resultat+(messageRX[i]*exp2(i));
    }
    //std::cout << "resultat : "<<resultat<<std::endl;
    usleep(2000);
    pthread_mutex_lock(&mutexAngle);
    anglePrecedent=angle;
    angle=resultat;
    if(abs(angle-anglePrecedent)>30000){
        angle=resultat-angleMax;
    }
    pthread_mutex_unlock(&mutexAngle);
}

void* TX(void* arg){

    // Changement politique et priorité
    /*struct sched_param params;
    params.sched_priority = 50;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement*/
    /*
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO)
    << "  prio=" << params.sched_priority << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"  <<std::endl;
    */
    sem_wait(semDemarrageCom);
    while(1){
        TXnaif();
    }
    return NULL;
}

int readRX(){
    return RXPin;
}



void FonctionnementNormal(){

}
/*
void* Correcteur(void* arg){
// Utilisation de sem_wait pour attendre que le sémaphore devienne disponible
sem_wait(semDemarrage);


// Etape 2 : Ziegler-Nichols
//int angleConsigne=(int)consigne;
angleConsigne = consigne;
//int Gain=(int)gain;
//int nvc = 0;
float Te=0.004;
float Tc=0.226;
Kp=0.3;
float Ti=0.83*Tc;
float Td=(Tc/8);
float Ki=1/Ti;
float Kd=1/Td;




DiviseurKp=0;
if(angleConsigne<=2500){
DiviseurKp=(0.0000016*exptt(angleConsigne,2)) + (0.0001136*exptt(angleConsigne,1)) + 0.0497947;//(0.00000000000049502762*exptt(consigne,4))- (0.00000000507315463776*exptt(consigne,3)) + (0.00001619760699757390*exptt(consigne,2 ))- (0.01213462811081350000*exptt(consigne,1))+ 5.60653903558871000000;
} else if (angleConsigne<=10000) {
DiviseurKp= (0.002*angleConsigne)+5;
} else {
DiviseurKp=32;
}


//Kp=Kp/25;//5000
//Kp = Kp/2.85;
Kp = Kp/DiviseurKp;
Ki=Ki/1;
Kd=Kd/10000;


float accumulIntegral=0;
ecartPrecedent=0;

float proportionnel=0;
float derive=0;
float integral=0;

float sortie = 0;
float DeltaT=0.004;

int compteurErreur = 0;



  while(1){
pthread_mutex_lock(&mutexAngle);
int angleCorrecteur=angle;
pthread_mutex_unlock(&mutexAngle);

ecart=angleConsigne-angleCorrecteur;

  int Erreur=0;
  if((sortie>500)&&(ecart==ecartPrecedent)){
  Erreur=1;
  }
  switch (Erreur) {
case 1: // Rotor bloqué
//PWM(0);
//perror("Erreur Rotor bloqué");
//std::cout << "Erreur Rotor bloqué "<<std::endl;
//return NULL;
compteurErreur++;
std::cout << "compteurErreur "<< compteurErreur  <<std::endl;
if(compteurErreur==(((int)PERIODE_ECHEC)/((int)PERIODE_CORRECTEUR))){
//Erreur = 1000;
PWM(0);
compteurErreur=0;
fprintf(stderr,"Erreur détectée, moteur arrêté, angle actuel : %f",angleCorrecteur);
std::cout << "angleCorrecteur "<< angleCorrecteur  <<std::endl;
std::cout << "compteurErreur "<< compteurErreur  <<std::endl;
return NULL;
}
break;
case 2: // Relais éteint
PWM(0);
std::cout << "Relais éteint "<<std::endl;
compteurErreur 1250
Erreur détectée, moteur arrêté, angle actuel : 8924
angleCorrecteur 8924
compteurErreur 0

return NULL;
  }
std::cout << "DiviseurKp : "<<DiviseurKp<<std::endl;

int PWM_FCT_Ecart = (int)((abs(ecart)/360)*1024);
integral=(Ki*Te*ecart)+(FacteurOubli*integral);

std::cout << "integral : "<<integral<<std::endl;
std::cout << "ecart : "<<ecart<<std::endl;

sortie = abs(Kp*(ecart + integral + ((Kd/Te) * (ecart-ecartPrecedent))));
std::cout << "sortie : "<<sortie<<std::endl;
//std::cout << "sortie : "<<sortie<<std::endl;
//sortie = Kp*(proportionnel + integral);

if(ecart>0){
initSR(1);
} else if(ecart<0){
initSR(0);
}
//PWM(sortie*ecart);
//std::cout << "PWM : "<<PWM_FCT_Ecart<<std::endl;

PWM(sortie);

ecartPrecedent = ecart;
usleep((int)PERIODE_CORRECTEUR);

}
return NULL;
}
*/
void finRelais(int choix){
    if(choix==0){
        digitalWrite(pwrRelPin, LOW);
        pinMode(pwrRelPin, INPUT);
    } else if(choix==1){
        digitalWrite(lgcRelPin, LOW);
        pinMode(lgcRelPin, INPUT);
    }
}

void handle_connection(int client_socket) {
    char buffer[BUFFER_SIZE];
    int received_value=1;
    // Receive the character string representing the non-signed integer
    ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_received > 0) {
        //buffer[bytes_received] = '\0';

        // Convert the character string to an integer
        received_value = atoi(buffer);
        printf("Received integer value: %d \n", received_value);
        angleConsigne =  received_value;
        ecart=received_value-angle;
        if(ecart<=2500){
            DiviseurKp=(0.0000016*exptt(ecart,2)) + (0.0001136*exptt(ecart,1)) + 0.0497947;//(0.00000000000049502762*exptt(consigne,4))- (0.00000000507315463776*exptt(consigne,3)) + (0.00001619760699757390*exptt(consigne,2 ))- (0.01213462811081350000*exptt(consigne,1))+ 5.60653903558871000000;
        } 
        else if (ecart<=10000) {
            DiviseurKp= (0.002*ecart)+5;
        } 
        else {
            DiviseurKp=32;
        }
        //Kp=Kp/25;//5000
        //Kp = Kp/2.85;
        Kp = 0.3/DiviseurKp;
    } 
    else {
        perror("Error receiving data");
    }
    close(client_socket);
}

void* comTCP(void* arg){
    struct sockaddr_in server_address, client_address;
    socklen_t client_address_len = sizeof(client_address);
    // Create socket
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    // Set up server address
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT);
    server_address.sin_addr.s_addr = INADDR_ANY;
    // Bind socket
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
        perror("Bind failed");
        close(server_socket);
        exit(EXIT_FAILURE);
    }
    // Listen for incoming connections
    if (listen(server_socket, 5) == -1) {
        perror("Listen failed");
        close(server_socket);
        exit(EXIT_FAILURE);
    }
    printf("Listening on port %d...\n", PORT);
    while (1) {
        // Accept incoming connection
        client_socket = accept(server_socket, (struct sockaddr *)&client_address, &client_address_len);
        if (client_socket == -1) {
            perror("Accept failed");
            continue;
        }
        printf("Accepted connection from %s:%d\n", inet_ntoa(client_address.sin_addr), ntohs(client_address.sin_port));
        // Handle the connection in a separate function
        handle_connection(client_socket);
    }
    close(server_socket);
   return NULL;
}

void* ChangeStatePin(void* arg){ // 20x plus vite que le fréquence du PID à 238HZ => 4760 Hz => 5000Hz => 200 µs => 10+190 µs
    pinMode(pinMasterRequest, OUTPUT);
    while(1){
        digitalWrite(pinMasterRequest, HIGH);
        usleep(100);
        digitalWrite(pinMasterRequest, LOW);
        usleep(100);
    }
    return 0;  // pointeur sur rien du tout
}

void* rtSoftTimerThread(void* arg) {
    // Changement politique et priorité
    struct sched_param params;
    params.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO)
    << "  prio=" << params.sched_priority << std::endl;

    // Configuration WiringPi
    int wiringPiSetupStatus = wiringPiSetup();
    if (wiringPiSetupStatus != 0) {
        std::cerr << "wiringPiSetupStatus=" << wiringPiSetupStatus << " ; exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    pinMode(clockPin, OUTPUT);
    pinMode(dbgPin, OUTPUT);
    bool pinHigh = false;

    // type long: d'après la commande "man clock_gettime"
    long timerPeriod_ns = 1000L * TIMER_PERIOD_US;

    // On boucle sur un temps pré-défini (le temps de faire les tests)
    struct timespec startTime, iterationStartTime, sleepEndTime;
    clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
    bool continueTimer = true;
    while (continueTimer) {
        // inc !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pthread_mutex_lock(&mutexINC);
        INC=(INC+1)%(ratioTemps+1);
        //std::cout << "INC " << INC << std::endl;
        pthread_mutex_unlock(&mutexINC);
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO expliquer pourquoi CLOCK_MONOTONIC_RAW est l'option de clock la + adaptéef
        digitalWrite(dbgPin, HIGH);  // pour mesure le temps approx de ce call OS lui-même
        clock_gettime(CLOCK_MONOTONIC_RAW, &iterationStartTime);
        digitalWrite(dbgPin, LOW);

        // Pour visualiser la clock facilement sur un oscilloscope
        pinHigh = !pinHigh;
        digitalWrite(clockPin, (int)pinHigh);

        // TODO doc: something can be done here


        // Condition de sortie: si le temps de run en secondes est dépassé
        if ((iterationStartTime.tv_sec - TIMER_RUN_TIME_S) > startTime.tv_sec)
            continueTimer = false;

        // Calcul de l'instant auquel on aimerait se réveiller, pour enchaîner directement
        // sur la boucle suivante. Attention à gérer correctement la structure timespec,
        // surtout les overflows de nanosecondes
        if (iterationStartTime.tv_nsec + timerPeriod_ns >= 1000000000L) {  // Si on passe à la seconde suivante
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns - 1000000000L;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec + 1;
        }
        else {  // Sinon, cas le + simple: on ajoute juste les nanosecondes à attendre
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec;
        }

        // TODO sleep_until, 2 modes (compensated or not)
        if (USE_COMPENSATED_SLEEP)
            compensatedSleepUntil(&sleepEndTime);
        else
            naiveSleepUntil(&sleepEndTime);
    }
    // Fin du thread
    digitalWrite(clockPin, LOW);
    digitalWrite(dbgPin, LOW);
    std::cout << "Thread RT a terminé" << std::endl;
    return 0;  // pointeur sur rien du tout
}

void* rtSoftTimerThreadCorrecteur(void* arg) {
    // Changement politique et priorité
    struct sched_param params;
    params.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO)
    << "  prio=" << params.sched_priority << std::endl;
    /******************************************************/
    /***************** code perso correcteur :  variables*******************/
    // Utilisation de sem_wait pour attendre que le sémaphore devienne disponible
    sem_wait(semDemarrage);
    // Etape 2 : Ziegler-Nichols
    //int angleConsigne=(int)consigne;
    angleConsigne = consigne;
    //int Gain=(int)gain;
    //int nvc = 0;
    float Te=0.004;
    float Tc=0.226;
    Kp=0.3;
    float Ti=0.83*Tc;
    float Td=(Tc/8);
    float Ki=1/Ti;
    float Kd=1/Td;

    DiviseurKp=0;
    if(angleConsigne<=2500){
        DiviseurKp=(0.0000016*exptt(angleConsigne,2)) + (0.0001136*exptt(angleConsigne,1)) + 0.0497947;//(0.00000000000049502762*exptt(consigne,4))- (0.00000000507315463776*exptt(consigne,3)) + (0.00001619760699757390*exptt(consigne,2 ))- (0.01213462811081350000*exptt(consigne,1))+ 5.60653903558871000000;
    } 
    else if (angleConsigne<=10000) {
        DiviseurKp= (0.002*angleConsigne)+5;
    } 
    else {
        DiviseurKp=32;
    }

    //Kp=Kp/25;//5000
    //Kp = Kp/2.85;
    Kp = Kp/DiviseurKp;
    Ki=Ki/1;
    Kd=Kd/10000;

    float accumulIntegral=0;
    ecartPrecedent=0;

    float proportionnel=0;
    float derive=0;
    float integral=0;

    float sortie = 0;
    float DeltaT=0.004;

    int compteurErreur = 0;
    /******************************************************/

    // type long: d'après la commande "man clock_gettime"
    //long timerPeriod_ns = 1000L * TIMER_PERIOD_US;
    long timerPeriod_ns = 1000L * PERIODE_CORRECTEUR;

    // On boucle sur un temps pré-défini (le temps de faire les tests)
    struct timespec startTime, iterationStartTime, sleepEndTime;
    clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
    bool continueTimer = true;

    int variableBoucle=0;
    while (continueTimer) {
        variableBoucle++;
        // inc !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        pthread_mutex_lock(&mutexINC);
        INC=(INC+1)%(ratioTemps+1);
        //std::cout << "INC " << INC << std::endl;
        pthread_mutex_unlock(&mutexINC);
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO expliquer pourquoi CLOCK_MONOTONIC_RAW est l'option de clock la + adaptée
        digitalWrite(dbgPin, HIGH);  // pour mesure le temps approx de ce call OS lui-même
        clock_gettime(CLOCK_MONOTONIC_RAW, &iterationStartTime);
        digitalWrite(dbgPin, LOW);

        // TODO doc: something can be done here
        /************************ Code correcteur PID perso ******************************/
        pthread_mutex_lock(&mutexAngle);
        int angleCorrecteur=angle;
        pthread_mutex_unlock(&mutexAngle);

        ecart=angleConsigne-angleCorrecteur;

        int Erreur=0;
        if((sortie>300)&&(abs(ecart)>10)&&(ecart==ecartPrecedent)){
            Erreur=1;
            if(ecart != ecartDixPrecedent){
                compteurErreur=0;
            }
        std::cout << "Erreur détectée !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"  <<std::endl;
        }
        switch (Erreur) {
            case 1: // Rotor bloqué
                //PWM(0);
                //perror("Erreur Rotor bloqué");
                //std::cout << "Erreur Rotor bloqué "<<std::endl;
                //return NULL;
                compteurErreur++;
                Erreur=0;
                std::cout << "compteurErreur "<< compteurErreur  <<std::endl;
                if(compteurErreur==(((int)PERIODE_ECHEC)/((int)PERIODE_CORRECTEUR))){
                    //Erreur = 1000;
                    PWM(0);
                    compteurErreur=0;
                    fprintf(stderr,"Erreur détectée, moteur arrêté, angle actuel : %d\n",angleCorrecteur);

                    std::cout << "angleCorrecteur "<< angleCorrecteur  <<std::endl;
                    std::cout << "compteurErreur "<< compteurErreur  <<std::endl;
                    return NULL;
                }
                break;
            case 2: // Relais éteint
                PWM(0);
                std::cout << "Relais éteint "<<std::endl;
                return NULL;
            }
        std::cout << "DiviseurKp : "<<DiviseurKp<<std::endl;

        int PWM_FCT_Ecart = (int)((abs(ecart)/360)*1024);
        integral=(Ki*Te*ecart)+(FacteurOubli*integral);
        /*
        if(ecart<100){
        integral-=10*Ki*Te*ecart;
        }
        else{
        integral+=Ki*Te*ecart;
        }
        */
        std::cout << "integral : "<<integral<<std::endl;
        std::cout << "ecart : "<<ecart<<std::endl;

        /*int integralLimite=500000;
        if(integral>=integralLimite){
        integral=integralLimite;
        //integral-=Ki*Te*ecart;
        }*/

        sortie = abs(Kp*(ecart + integral + ((Kd/Te) * (ecart-ecartPrecedent))));
        std::cout << "sortie : "<<sortie<<std::endl;
        //std::cout << "sortie : "<<sortie<<std::endl;
        //sortie = Kp*(proportionnel + integral);

        if(ecart>0){
            initSR(1);
        } 
        else if(ecart<0){
            initSR(0);
        }
        //PWM(sortie*ecart);
        //std::cout << "PWM : "<<PWM_FCT_Ecart<<std::endl;

        PWM(sortie);
        /*
        if(abs(ecart)>10){
        PWM(sortie);
        } else {
        PWM(0);
        }
        */
        ecartPrecedent = ecart;
        if(variableBoucle%5==0){
            ecartDixPrecedent = ecart;
        }
        //usleep((int)PERIODE_CORRECTEUR);
        /********************************************************************/

        // Calcul de l'instant auquel on aimerait se réveiller, pour enchaîner directement
        // sur la boucle suivante. Attention à gérer correctement la structure timespec,
        // surtout les overflows de nanosecondes
        if (iterationStartTime.tv_nsec + timerPeriod_ns >= 1000000000L) {  // Si on passe à la seconde suivante
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns - 1000000000L;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec + 1;
        }
        else {  // Sinon, cas le + simple: on ajoute juste les nanosecondes à attendre
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec;
        }

        // TODO sleep_until, 2 modes (compensated or not)
        if (USE_COMPENSATED_SLEEP)
            compensatedSleepUntil(&sleepEndTime);
        else
            naiveSleepUntil(&sleepEndTime);
    }
    // Fin du thread
    digitalWrite(clockPin, LOW);
    digitalWrite(dbgPin, LOW);
    std::cout << "Thread RT a terminé" << std::endl;
    return 0;  // pointeur sur rien du tout
}


void* clockTimer(void* arg) {
    // Changement politique et priorité
    struct sched_param params;
    params.sched_priority = 50;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO)<< "  prio=" << params.sched_priority << std::endl;
    bool continueTimer = true;
    /*
    while(continueTimer){
    TX();
    }
    */
    return NULL;
}

void timespecDiff(const struct timespec* start, const struct timespec* end, struct timespec* duration) {
    // TODO gérer diffs > 1s
    if (start->tv_sec < end->tv_sec)
        duration->tv_nsec = end->tv_nsec + (1000000000L - start->tv_nsec);
    else
        duration->tv_nsec = end->tv_nsec - start->tv_nsec;
}

// Solution dite "naïve", qui ne prend pas en compte les latences de l'OS (même si RTOS)
void naiveSleepUntil(struct timespec* sleepEndTime) {
    struct timespec sleepStartTime, sleepDuration;
    sleepDuration.tv_sec = 0;
    clock_gettime(CLOCK_MONOTONIC_RAW, &sleepStartTime);
    timespecDiff(&sleepStartTime, sleepEndTime, &sleepDuration);
    nanosleep(&sleepDuration, 0);  // TODO gérer erreurs et remaining time
}

// Solution améliorée
void compensatedSleepUntil(struct timespec* sleepEndTime) {
    // TODO define du temps pris par clock_gettime
    struct timespec sleepStartTime, sleepCompensatedEndTime, sleepDuration;
    sleepDuration.tv_sec = 0;
    clock_gettime(CLOCK_MONOTONIC_RAW, &sleepStartTime);
    const long osMaxLatency_ns = OS_MAX_LATENCY_US * 1000L;
    if (sleepEndTime->tv_nsec < osMaxLatency_ns) {
        sleepCompensatedEndTime.tv_sec = sleepEndTime->tv_sec - 1;
        sleepCompensatedEndTime.tv_nsec =  1000000000L + sleepEndTime->tv_nsec - osMaxLatency_ns;
    }
    else {
        sleepCompensatedEndTime.tv_nsec = sleepEndTime->tv_nsec - osMaxLatency_ns;
        sleepCompensatedEndTime.tv_sec = sleepEndTime->tv_sec;
    }
    timespecDiff(&sleepStartTime, &sleepCompensatedEndTime, &sleepDuration);
    nanosleep(&sleepDuration, 0);  // TODO gérer erreurs et remaining time

    // à ce stade, on sait qu'on est resté endormi trop peu de temps
    // --> attente active (mais avec calls à l'OS, temps de calcul du kernel non-déterministe)
    struct timespec activePauseIterationStartTime, activePauseDuration;
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &activePauseIterationStartTime);
        timespecDiff(&activePauseIterationStartTime, sleepEndTime, &activePauseDuration);
    }
    while (activePauseDuration.tv_nsec > CLOCK_GETTIME_AVG_DURATION_NS);
}
