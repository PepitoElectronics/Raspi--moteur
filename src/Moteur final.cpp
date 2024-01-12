//============================================================================
// Name        : Moteur.cpp
// Author      : Olivier Van Roost - Juan Alvarez Lopez
// Version     :
// Copyright   : Your copyright notice
//============================================================================

/**************************** Bibliothèques ****************************************/
#include <iostream>      // Entrée/sortie standard
#include "unistd.h"      // API POSIX pour le système d'exploitation
#include <pthread.h>     // Threads POSIX
#include <time.h>        // Fonctions liées au temps
#include "wiringPi.h"    // Bibliothèque WiringPi pour les GPIO sur Raspberry Pi
#include <math.h>        // Fonctions mathématiques
#include <stdio.h>       // Opérations d'entrée/sortie standard
#include <sys/socket.h>  // Programmation socket
#include <stdlib.h>      // Bibliothèque standard
#include <string.h>      // Fonctions de manipulation de chaînes de caractères
#include <netinet/in.h>  // Famille d'adresses Internet
#include <semaphore.h>   // Synchronisation avec des sémaphores
#include <signal.h>      // Gestion des signaux (pour la fermeture du programme)
// TCP
#include <arpa/inet.h>   // Définitions pour les opérations Internet
#include <errno.h>       // Définitions des numéros d'erreur

/**************************** Defines ****************************************/
#define TIMER_RUN_TIME_S (5 * 60) // temps approx d'exécution du programme, en secondes
#define TIMER_PERIOD_US 500       // période du timer, en microsecondes
#define OS_MAX_LATENCY_US 130     // latence max de l'OS temps-réel (cyclictest), en microsecondes
// En moyenne 2 microsecondes, mais pointe mesurée à 130microsecondes...
// probablement arrivée dans un cas où le kernel lui-même était temporairement sur-chargé
#define CLOCK_GETTIME_AVG_DURATION_NS (2100L)
#define USE_COMPENSATED_SLEEP 1 // compensation de latence, ou sleep naïf
/* Comparaison des 2 pour un rpi 3 A+ kernel 4.19.71-rt24-v7+; compilation release -03, prio FIFO 99
 * mesures à l'oscilloscope pendant 5 minutes, pour un timer à 2kHz (500us), mesures en us
 *
 *                          avg       min      max
 * sans compensation:       526       481      708
 * avec compensation:       500       479      638
 */
// Taille max du buffer d'envoi pour la lisaison série
#define nBitsAEnvoyer 20 
// Gain de départ pour appliquer la méthode de Ziegler-Nichols
#define gain 6 
// Définition du port à utiliser pour la communication
#define PORT 1883
// Taille du tampon (buffer) pour la communication
#define BUFFER_SIZE 32
// Période propre au correcteur en microsecondes (us)
#define PERIODE_CORRECTEUR 4000
// Période maximale autorisée avec persistance d'erreur en microsecondes (us)
#define PERIODE_ECHEC 5000000
/****************************  PROTOTYPES  *********************************************/
void *rtSoftTimerThread(void *arg);
void *rtSoftTimerThreadCorrecteur(void *arg);
void *clockTimer(void *arg);
void *comTCP(void *arg);
void *TX(void *arg);
void *Correcteur(void *arg);
void *ChangeStatePin(void *arg);
void naiveSleepUntil(struct timespec *sleepEndTime);
void compensatedSleepUntil(struct timespec *sleepEndTime);
void initRelais();
void initSR(int sens);
void initRX();
void initTX();
void finRelais(int choix);
void PWM(int rc);
void TXnaif();
int exp2(int exponent);
int exptt(int base, int exponent);
void generalSignalHandler(int);
void terminerProgramme();
/******************** Câblage WiringPi ******************************/
const int clockPin = 0;          // Numéro de broche WiringPi pour le signal d'horloge
const int dbgPin = 1;            // Numéro de broche WiringPi pour le débogage
const int lgcRelPin = 11;        // Numéro de broche WiringPi pour le relais logique
const int pwrRelPin = 31;        // Numéro de broche WiringPi pour le relais d'alimentation
const int RXPin = 13;            // Numéro de broche WiringPi pour la broche de réception (RX)
const int TXPin = 14;            // Numéro de broche WiringPi pour la broche de transmission (TX)
const int pwmPin = 26;           // Numéro de broche WiringPi pour la broche PWM
const int sr1Pin = 6;            // Numéro de broche WiringPi pour la broche liée au sens de rotation 1
const int sr2Pin = 10;           // Numéro de broche WiringPi pour la broche liée au sens de rotation 2
const int pinMasterRequest = 15; // Numéro de broche WiringPi pour la demande de maître

/************** Variables du programme ******************/
int consigne = 0; // Consigne initiale (hard-codée) en degré
float DiviseurKp; // Facteur de division du kp
float Kp; // Gain proportionnel du correcteur PID mixte
float FacteurOubli = 0.9985; // Facteur utilisé pour permettre de vider la composante intégrale
int messageRX[nBitsAEnvoyer]; // Contient les caractères envoyés dans la trame circulant sur la liaison série
int angle = 0; // Contient le résultat envoyé sur la liaison série (angle équivalent)
int anglePrecedent = 0; // Penultième angle reçu 
int angleMax = 65535; // Limite maximale en angle pour le système
int ecart; // valeur de l'écrat utilisé pour le PID
int ecartPrecedent; // Penultième écart calculé 
int ecartDixPrecedent;// Ancien écart calculé (10ème valeur précédente) 
int angleConsigne = 0;// Anglme de consigne 
int server_socket, client_socket; // Socket pour la communication TCP/IP
int utime = 600000;

// Mutex de protection des variables partagées
pthread_mutex_t mutexAngle;
pthread_mutex_t mutexAngleConsigne;
pthread_mutex_t mutexINC;

// Sémaphores de synchronisation des threads
sem_t *semDemarrage;
sem_t *semDemarrageCom;
/************************** Main *********************************/
// Fonction principale
int main()
{
    // Gestion de SIGTERM
// Déclaration d'une structure sigaction pour la gestion des signaux
struct sigaction signalAction;
// Initialisation du masque de signaux à ignorer pendant l'exécution du gestionnaire de signal
sigemptyset(&signalAction.sa_mask);
// Configuration des options de la struct sigaction
signalAction.sa_flags = 0;
// Attribution de la fonction de gestion des signaux (generalSignalHandler) à sa_handler
signalAction.sa_handler = &generalSignalHandler;
// Tentative d'intercepter le signal SIGTERM en utilisant la fonction sigaction
if (sigaction(SIGTERM, &signalAction, NULL) == -1)
{
    // Affiche un message d'erreur si l'interception du signal échoue
    std::cerr << "Impossible d'intercepter SIGTERM !" << std::endl;
}

    // Allocation dynamique de la mémoire pour les sémaphores
    sem_t *semDemarrage = (sem_t *)malloc(sizeof(sem_t));
    sem_t *semDemarrageCom = (sem_t *)malloc(sizeof(sem_t));

    sem_init(semDemarrage, 0, 0);
    sem_init(semDemarrageCom, 0, 0);

    // Initialisation de WiringPi
    wiringPiSetup();
    pinMode(pwmPin, PWM_OUTPUT);

    std::cout << "Real-Time software Timer on Raspberry Pi 3" << std::endl;

    // TIDs pour les threads
    pthread_t rtThreadTid, clockThreadTid, correcteurThreadTid, TCPTid, TXTid;

    // Configuration et lancement des threads
    pthread_attr_t threadAttributes;
    pthread_attr_init(&threadAttributes);
    pthread_attr_setdetachstate(&threadAttributes, PTHREAD_CREATE_JOINABLE);

    pthread_create(&rtThreadTid, &threadAttributes, &rtSoftTimerThread, 0);// Thread du rtSoftTimerThread
    pthread_create(&clockThreadTid, &threadAttributes, &clockTimer, 0);// Thread de la clock avec l'ordinateur
    pthread_create(&correcteurThreadTid, &threadAttributes, &rtSoftTimerThreadCorrecteur, 0); // Thread du correcteur
    pthread_create(&TCPTid, &threadAttributes, &comTCP, 0); // Thread de la communication TCP avec l'ordinateur
    pthread_create(&TXTid, &threadAttributes, &TX, 0);// Thread de la communication série avec l'ordinateur

    pthread_attr_destroy(&threadAttributes);

    // Autres initialisations
    initRX();
    initTX();

    pthread_t thread_MasterRequest;
    pthread_create(&thread_MasterRequest, NULL, &ChangeStatePin, NULL);

    // Plus de configurations/initiations...

    // Poste les sémaphores pour démarrer les threads
    sem_post(semDemarrage);
    sem_post(semDemarrageCom);

    // Boucle principale
    while (1)
    {
        usleep(60);
    }

    // Attente la fin des threads avec join
    pthread_join(rtThreadTid, 0);

    // Nettoyage des ressources
    free(semDemarrage);
    free(semDemarrageCom);

    return 0;
}

// Fonction pour initialiser les broches de décalage de registre
// en fonction du sens spécifié (0 pour un sens, 1 pour l'autre sens)
void initSR(int sens)
{
    // Vérifie le sens spécifié
    if (sens == 0)
    {
        // Configuration de la broche sr1Pin en mode sortie
        pinMode(sr1Pin, OUTPUT);
        // Positionnement de la broche sr1Pin à un niveau logique élevé (HIGH)
        digitalWrite(sr1Pin, HIGH);
        // Configuration de la broche sr2Pin en mode sortie
        pinMode(sr2Pin, OUTPUT);
        // Positionnement de la broche sr2Pin à un niveau logique bas (LOW)
        digitalWrite(sr2Pin, LOW);
    }
    else
    {
        // Configuration de la broche sr2Pin en mode sortie
        pinMode(sr2Pin, OUTPUT);
        // Positionnement de la broche sr2Pin à un niveau logique élevé (HIGH)
        digitalWrite(sr2Pin, HIGH);
        // Configuration de la broche sr1Pin en mode sortie
        pinMode(sr1Pin, OUTPUT);
        // Positionnement de la broche sr1Pin à un niveau logique bas (LOW)
        digitalWrite(sr1Pin, LOW);
    }
}


// Fonction pour configurer la modulation de largeur d'impulsion (PWM)
// en ajustant la fréquence et le rapport cyclique en fonction du paramètre rc
void PWM(int rc)
{
    // Définir le mode PWM sur mark-space (MS) pour une meilleure précision
    pwmSetMode(PWM_MODE_MS);
    // Définir la fréquence de PWM (50 kHz dans cet exemple)
    pwmSetClock(384);  // La fréquence de base de 19.2 MHz divisée par 384 donne 50 kHz
    // Définir la plage de valeurs pour le rapport cyclique (de 0 à 1023)
    pwmSetRange(1024);
    // Définir le rapport cyclique initial en fonction du paramètre rc
    pwmWrite(pwmPin, rc);
}


// Fonction pour initialiser les relais
void initRelais()
{
    // Configuration des broches des relais en mode sortie
    pinMode(lgcRelPin, OUTPUT);
    pinMode(pwrRelPin, OUTPUT);
    // Pause d'une seconde pour permettre aux relais de s'initialiser
    sleep(1);
    // Activation du relais logique (mise à un)
    digitalWrite(lgcRelPin, HIGH);
    // Pause d'une seconde pour permettre aux relais de s'initialiser
    sleep(1);
    // Activation du relais d'alimentation (mise à un)
    digitalWrite(pwrRelPin, HIGH);
    // Affichage d'un message indiquant que l'initialisation des relais est terminée
    std::cout << "Relais OK " << std::endl;
}

void initRX()
{
    pinMode(RXPin, INPUT);
}

void initTX()
{
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

int exp2(int exponent)
{
    int base = 2;
    int result_multiply = 1;

    for (int i = 0; i < exponent; ++i)
    {
        result_multiply *= base;
    }
    return result_multiply;
}

int exptt(int base, int exponent)
{
    int result_multiply = 1;
    for (int i = 0; i < exponent; ++i)
    {
        result_multiply *= base;
    }
    return result_multiply;
}

void TXnaif()
{
    bool pinHigh = false;
    digitalWrite(TXPin, LOW);
    // int local_consigne = consigne;
    int stateRX = 1;
    while (stateRX != 0)
    {
        stateRX = digitalRead(RXPin);
        // std::cout << "Attend RX" << std::endl;
        usleep(1);
    }
    int nBitsEnvoye = 0;
    while (nBitsEnvoye < nBitsAEnvoyer)
    {
        usleep(50);
        digitalWrite(TXPin, HIGH);
        // std::cout << "Front montant" << std::endl;
        usleep(25);
        messageRX[nBitsEnvoye] = digitalRead(RXPin);
        nBitsEnvoye += 1;
        usleep(25);
        digitalWrite(TXPin, LOW);
        // std::cout << "Front descendant" << std::endl;
    }
    // std::cout << "Fin message" << std::endl;
    digitalWrite(TXPin, HIGH);

    int resultat = 0;
    for (int i = 0; i < 16; i++)
    {
        // std::cout << "messageRX case n° "<< i<< " : " << messageRX[i] <<std::endl;
        resultat = resultat + (messageRX[i] * exp2(i));
    }
    // std::cout << "resultat : "<<resultat<<std::endl;
    usleep(2000);
    pthread_mutex_lock(&mutexAngle);
    anglePrecedent = angle;
    angle = resultat;
    if (abs(angle - anglePrecedent) > 30000)
    {
        angle = resultat - angleMax;
    }
    pthread_mutex_unlock(&mutexAngle);
}

void *TX(void *arg)
{

    // Changement politique et priorité
    struct sched_param params;
    params.sched_priority = 50;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement*/

    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO)
              << "  prio=" << params.sched_priority << "!!!!!!!!" << std::endl;

    sem_wait(semDemarrageCom);
    while (1)
    {
        TXnaif();
    }
    return NULL;
}

int readRX()
{
    return RXPin;
}

void FonctionnementNormal()
{
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

void finRelais(int choix)
{
    if (choix == 0)
    {
        digitalWrite(pwrRelPin, LOW);
        pinMode(pwrRelPin, INPUT);
    }
    else if (choix == 1)
    {
        digitalWrite(lgcRelPin, LOW);
        pinMode(lgcRelPin, INPUT);
    }
}

void handle_connection(int client_socket)
{
    char buffer[BUFFER_SIZE];
    int received_value = 1;
    // Receive the character string representing the non-signed integer
    ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_received > 0)
    {
        // buffer[bytes_received] = '\0';

        // Convert the character string to an integer
        received_value = atoi(buffer);
        printf("Received integer value: %d \n", received_value);
        angleConsigne = received_value;
        ecart = received_value - angle;
        if (ecart <= 2500)
        {
            DiviseurKp = (0.0000016 * exptt(ecart, 2)) + (0.0001136 * exptt(ecart, 1)) + 0.0497947; //(0.00000000000049502762*exptt(consigne,4))- (0.00000000507315463776*exptt(consigne,3)) + (0.00001619760699757390*exptt(consigne,2 ))- (0.01213462811081350000*exptt(consigne,1))+ 5.60653903558871000000;
        }
        else if (ecart <= 10000)
        {
            DiviseurKp = (0.002 * ecart) + 5;
        }
        else
        {
            DiviseurKp = 32;
        }
        // Kp=Kp/25;//5000
        // Kp = Kp/2.85;
        Kp = 0.3 / DiviseurKp;
    }
    else
    {
        perror("Error receiving data");
    }
    close(client_socket);
}

void *comTCP(void *arg)
{
    struct sockaddr_in server_address, client_address;
    socklen_t client_address_len = sizeof(client_address);
    // Create socket
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    // Set up server address
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT);
    server_address.sin_addr.s_addr = INADDR_ANY;
    // Bind socket
    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) == -1)
    {
        perror("Bind failed");
        close(server_socket);
        exit(EXIT_FAILURE);
    }
    // Listen for incoming connections
    if (listen(server_socket, 5) == -1)
    {
        perror("Listen failed");
        close(server_socket);
        exit(EXIT_FAILURE);
    }
    printf("Listening on port %d...\n", PORT);
    while (1)
    {
        // Accept incoming connection
        client_socket = accept(server_socket, (struct sockaddr *)&client_address, &client_address_len);
        if (client_socket == -1)
        {
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

void *ChangeStatePin(void *arg)
{ // 20x plus vite que le fréquence du PID à 238HZ => 4760 Hz => 5000Hz => 200 µs => 10+190 µs
    pinMode(pinMasterRequest, OUTPUT);
    while (1)
    {
        digitalWrite(pinMasterRequest, HIGH);
        usleep(100);
        digitalWrite(pinMasterRequest, LOW);
        usleep(100);
    }
    return 0; // pointeur sur rien du tout
}

void *rtSoftTimerThread(void *arg)
{
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
    if (wiringPiSetupStatus != 0)
    {
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
    while (continueTimer)
    {

        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // TODO expliquer pourquoi CLOCK_MONOTONIC_RAW est l'option de clock la + adaptéef
        digitalWrite(dbgPin, HIGH); // pour mesure le temps approx de ce call OS lui-même
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
        if (iterationStartTime.tv_nsec + timerPeriod_ns >= 1000000000L)
        { // Si on passe à la seconde suivante
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns - 1000000000L;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec + 1;
        }
        else
        { // Sinon, cas le + simple: on ajoute juste les nanosecondes à attendre
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
    return 0; // pointeur sur rien du tout
}

void *rtSoftTimerThreadCorrecteur(void *arg)
{
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

    float Te = 0.004;
    float Tc = 0.226;
    float Ti = 0.83 * Tc;
    float Td = (Tc / 8);
    float Ki = 1 / Ti;
    float Kd = 1 / Td;

    float accumulIntegral = 0;
    float proportionnel = 0;
    float derive = 0;
    float integral = 0;
    float sortie = 0;
    float DeltaT = 0.004;
    int compteurErreur = 0;
    int variableBoucle = 0;

    Kp = 0.3;
    angleConsigne = consigne;
    DiviseurKp = 0;
    Kp = Kp / DiviseurKp;
    Ki = Ki / 1;
    Kd = Kd / 10000;
    ecartPrecedent = 0;

    if (angleConsigne <= 2500)
    {
        DiviseurKp = (0.0000016 * exptt(angleConsigne, 2)) + (0.0001136 * exptt(angleConsigne, 1)) + 0.0497947; //(0.00000000000049502762*exptt(consigne,4))- (0.00000000507315463776*exptt(consigne,3)) + (0.00001619760699757390*exptt(consigne,2 ))- (0.01213462811081350000*exptt(consigne,1))+ 5.60653903558871000000;
    }
    else if (angleConsigne <= 10000)
    {
        DiviseurKp = (0.002 * angleConsigne) + 5;
    }
    else
    {
        DiviseurKp = 32;
    }
    /******************************************************/
    // type long: d'après la commande "man clock_gettime"
    // long timerPeriod_ns = 1000L * TIMER_PERIOD_US;
    long timerPeriod_ns = 1000L * PERIODE_CORRECTEUR;
    // On boucle sur un temps pré-défini (le temps de faire les tests)
    struct timespec startTime, iterationStartTime, sleepEndTime;
    clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
    bool continueTimer = true;

    while (continueTimer)
    {
        variableBoucle++;
        // TODO expliquer pourquoi CLOCK_MONOTONIC_RAW est l'option de clock la + adaptée
        digitalWrite(dbgPin, HIGH); // pour mesure le temps approx de ce call OS lui-même
        clock_gettime(CLOCK_MONOTONIC_RAW, &iterationStartTime);
        digitalWrite(dbgPin, LOW);

        // TODO doc: something can be done here
        /************************ Code correcteur PID perso ******************************/
        pthread_mutex_lock(&mutexAngle);
        int angleCorrecteur = angle;
        pthread_mutex_unlock(&mutexAngle);

        ecart = angleConsigne - angleCorrecteur;

        int Erreur = 0;
        if ((sortie > 300) && (abs(ecart) > 10) && (ecart == ecartPrecedent))
        {
            Erreur = 1;
            if (ecart != ecartDixPrecedent)
            {
                compteurErreur = 0;
            }
            std::cout << "Erreur détectée !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        }
        switch (Erreur)
        {
        case 1: // Rotor bloqué
            compteurErreur++;
            Erreur = 0;
            std::cout << "compteurErreur " << compteurErreur << std::endl;
            if (compteurErreur == (((int)PERIODE_ECHEC) / ((int)PERIODE_CORRECTEUR)))
            {
                // Erreur = 1000;
                PWM(0);
                compteurErreur = 0;
                fprintf(stderr, "Erreur détectée, moteur arrêté, angle actuel : %d\n", angleCorrecteur);

                std::cout << "angleCorrecteur " << angleCorrecteur << std::endl;
                std::cout << "compteurErreur " << compteurErreur << std::endl;
                return NULL;
            }
            break;
        case 2: // Relais éteint
            PWM(0);
            std::cout << "Relais éteint " << std::endl;
            return NULL;
        }
        std::cout << "DiviseurKp : " << DiviseurKp << std::endl;
        int PWM_FCT_Ecart = (int)((abs(ecart) / 360) * 1024);
        integral = (Ki * Te * ecart) + (FacteurOubli * integral);
        std::cout << "integral : " << integral << std::endl;
        std::cout << "ecart : " << ecart << std::endl;
        sortie = abs(Kp * (ecart + integral + ((Kd / Te) * (ecart - ecartPrecedent))));
        std::cout << "sortie : " << sortie << std::endl;
        // std::cout << "sortie : "<<sortie<<std::endl;
        // sortie = Kp*(proportionnel + integral);

        if (ecart > 0)
        {
            initSR(1);
        }
        else if (ecart < 0)
        {
            initSR(0);
        }
        PWM(sortie);
        ecartPrecedent = ecart;
        if (variableBoucle % 5 == 0)
        {
            ecartDixPrecedent = ecart;
        }
        // usleep((int)PERIODE_CORRECTEUR);
        /********************************************************************/
        // Calcul de l'instant auquel on aimerait se réveiller, pour enchaîner directement
        // sur la boucle suivante. Attention à gérer correctement la structure timespec,
        // surtout les overflows de nanosecondes
        if (iterationStartTime.tv_nsec + timerPeriod_ns >= 1000000000L)
        { // Si on passe à la seconde suivante
            sleepEndTime.tv_nsec = iterationStartTime.tv_nsec + timerPeriod_ns - 1000000000L;
            sleepEndTime.tv_sec = iterationStartTime.tv_sec + 1;
        }
        else
        { // Sinon, cas le + simple: on ajoute juste les nanosecondes à attendre
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
    return 0; // pointeur sur rien du tout
}

void *clockTimer(void *arg)
{
    // Changement politique et priorité
    struct sched_param params;
    params.sched_priority = 50;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &params);
    // Vérification du changement
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &params);
    std::cout << "Thread RT:  SCHED_FIFO=" << std::boolalpha << (policy == SCHED_FIFO) << "  prio=" << params.sched_priority << std::endl;
    bool continueTimer = true;
    /*
    while(continueTimer){
    TX();
    }
    */
    return NULL;
}

void timespecDiff(const struct timespec *start, const struct timespec *end, struct timespec *duration)
{
    // TODO gérer diffs > 1s
    if (start->tv_sec < end->tv_sec)
        duration->tv_nsec = end->tv_nsec + (1000000000L - start->tv_nsec);
    else
        duration->tv_nsec = end->tv_nsec - start->tv_nsec;
}

// Solution dite "naïve", qui ne prend pas en compte les latences de l'OS (même si RTOS)
void naiveSleepUntil(struct timespec *sleepEndTime)
{
    struct timespec sleepStartTime, sleepDuration;
    sleepDuration.tv_sec = 0;
    clock_gettime(CLOCK_MONOTONIC_RAW, &sleepStartTime);
    timespecDiff(&sleepStartTime, sleepEndTime, &sleepDuration);
    nanosleep(&sleepDuration, 0); // TODO gérer erreurs et remaining time
}

// Solution améliorée
void compensatedSleepUntil(struct timespec *sleepEndTime)
{
    // TODO define du temps pris par clock_gettime
    struct timespec sleepStartTime, sleepCompensatedEndTime, sleepDuration;
    sleepDuration.tv_sec = 0;
    clock_gettime(CLOCK_MONOTONIC_RAW, &sleepStartTime);
    const long osMaxLatency_ns = OS_MAX_LATENCY_US * 1000L;
    if (sleepEndTime->tv_nsec < osMaxLatency_ns)
    {
        sleepCompensatedEndTime.tv_sec = sleepEndTime->tv_sec - 1;
        sleepCompensatedEndTime.tv_nsec = 1000000000L + sleepEndTime->tv_nsec - osMaxLatency_ns;
    }
    else
    {
        sleepCompensatedEndTime.tv_nsec = sleepEndTime->tv_nsec - osMaxLatency_ns;
        sleepCompensatedEndTime.tv_sec = sleepEndTime->tv_sec;
    }
    timespecDiff(&sleepStartTime, &sleepCompensatedEndTime, &sleepDuration);
    nanosleep(&sleepDuration, 0); // TODO gérer erreurs et remaining time

    // à ce stade, on sait qu'on est resté endormi trop peu de temps
    // --> attente active (mais avec calls à l'OS, temps de calcul du kernel non-déterministe)
    struct timespec activePauseIterationStartTime, activePauseDuration;
    do
    {
        clock_gettime(CLOCK_MONOTONIC_RAW, &activePauseIterationStartTime);
        timespecDiff(&activePauseIterationStartTime, sleepEndTime, &activePauseDuration);
    } while (activePauseDuration.tv_nsec > CLOCK_GETTIME_AVG_DURATION_NS);
}

// Création et initialisation de la structure POSIX
void generalSignalHandler(int signal)
{
    if (signal == SIGTERM)
    {
        std::cout << std::endl
                  << "Exécution de terminerProgramme() en cours..." << std::endl;
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
