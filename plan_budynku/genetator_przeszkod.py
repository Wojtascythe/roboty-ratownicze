import random
import copy
import os

CZLOWIEK='H'
PRZESZKODA='O'
SCHODY='S'
SCIANA='W'
PUSTO='E'


#pobiera dane
def give_pietra():
    ilosc_pieter=input("Podaj liczbe pieter: ")

    ilosc_pieter=int(ilosc_pieter)
    return ilosc_pieter

#pobiera nazwe
def give_file_dane():
    nazwa_pliku=input("Podaj nazwe folderu zawierajacego mapy: ")

    ilosc_ludzi=input("Podaj liczbe ludzi: ")
    ilosc_przeszkod=input("Podaj liczbe przeszkod: ")

    ilosc_ludzi=int(ilosc_ludzi)
    ilosc_przeszkod=int(ilosc_przeszkod)
    return nazwa_pliku, ilosc_ludzi, ilosc_przeszkod

#czyta z pliku po linii
def read_file(nazwa):
    plik=open(nazwa, 'r')
    try:
        zawartosc=plik.readlines()
    finally:
        plik.close()
    return zawartosc

#pomija linie z "<"
def separator(zawartosc_pliku):
    mapa=[]

    for i in range(len(zawartosc_pliku)):
        if zawartosc_pliku[i][0]!='1' and zawartosc_pliku[i][0]!='0':
            continue
        mapa.append(zawartosc_pliku[i])

#podzial do tablicy 2 wymiarowej
    tab=[]
    for i in range(len(mapa)):
        #mapa[i]=mapa[i].rstrip('\n')
        tab.append(mapa[i].split(','))
    return tab

def print_in_terminal(macierz):
    for i in range(len(macierz)):
        for j in range(len(macierz[i])):
            print(macierz[i][j], end='')

#zmiana 0 na 4-czlowiek 3-przeszkoda
def add_object(tab, ilosc, obiekt):
    j=0
    for i in range(ilosc+j):
        x=random.randint(1, len(tab)-1)
        y=random.randint(1, len(tab[0])-2)

        if tab[x][y]=='E':
            tab[x][y]=obiekt
        else:
            j+=1
    return tab

def switch_object(tab):
    for szerokosc in range(len(tab)):
        for wysokosc in range(len(tab[0])-1):
            if tab[szerokosc][wysokosc]=='0':
                tab[szerokosc][wysokosc]='E'

            elif tab[szerokosc][wysokosc]=='2' or tab[szerokosc][wysokosc]=='3':
                tab[szerokosc][wysokosc]='S'

            else:
                tab[szerokosc][wysokosc]='W'

    return tab

def give_name_save():
    nazwa_pliku=input("Podaj nazwe pliku do zapisu: ")
    return nazwa_pliku

def save(nazwa, ilosc_pieter, tryb, pietro, ilosc_ludzi, ilosc_przeszkod, tablica_przeszkody, tablica_czysta):
    n_przeszkody=nazwa+"_przeszkody.rmap"
    n_czysta=nazwa+"_czysta.rmap"

    plik = open(n_przeszkody, tryb)
    if tryb == 'w':
        plik.write(str(ilosc_pieter))
    plik.write('\n')
    plik.write('\n')
    plik.write(str(pietro))
    plik.write('\n')
    tmp=str(ilosc_ludzi)+" "+str(ilosc_przeszkod)
    plik.write(tmp)
    plik.write('\n')
    for i in range(len(tablica_przeszkody)-1):
        for j in range(len(tablica_przeszkody[0])):
            if j==0:
                plik.write(str(tablica_przeszkody[i][j]))
            else:
                plik.write(" "+str(tablica_przeszkody[i][j]))
    for j in range(len(tablica_przeszkody[0])-1):
        if j==0:
            plik.write(str(tablica_przeszkody[len(tablica_przeszkody)-1][j]))
        else:
            plik.write(" "+str(tablica_przeszkody[len(tablica_przeszkody)-1][j]))

    plik.close()

    plik = open(n_czysta, tryb)
    if tryb == 'w':
        plik.write(str(ilosc_pieter))
    plik.write('\n')
    plik.write('\n')
    plik.write(str(pietro))
    plik.write('\n')
    tmp = str(ilosc_ludzi) + " " + str(ilosc_przeszkod)
    plik.write(tmp)
    plik.write('\n')
    for i in range(len(tablica_czysta) - 1):
        for j in range(len(tablica_czysta[0])):
            if j == 0:
                plik.write(str(tablica_czysta[i][j]))
            else:
                plik.write(" " + str(tablica_czysta[i][j]))
    for j in range(len(tablica_czysta[0]) - 1):
        if j == 0:
            plik.write(str(tablica_czysta[len(tablica_czysta) - 1][j]))
        else:
            plik.write(" " + str(tablica_czysta[len(tablica_czysta) - 1][j]))
    plik.close()

def listDirectory(directory):
    fileList = [os.path.normcase(f) for f in os.listdir(directory)]    #(1) (2)
    return fileList

def przetworzenie_danych_wejsciowych(ludzie, przeszkody, pietra):
    tab_ludzie=[]
    tab_przeszkody=[]

    for i in range(pietra):
        tab_ludzie.append(random.randint(0, ludzie))
        ludzie=ludzie-tab_ludzie[i]

        tab_przeszkody.append(random.randint(0, przeszkody))
        przeszkody=przeszkody-tab_przeszkody[i]

    return tab_ludzie, tab_przeszkody

def main():
    #pietra=give_pietra()
    nazwa, ludzie, przeszkody=give_file_dane()
    nazwa_do_zapisu = give_name_save()

    lista_plikow=listDirectory(nazwa)
    pietra=len(lista_plikow)
    tab_ludzie, tab_przeszkody=przetworzenie_danych_wejsciowych(ludzie, przeszkody, pietra)

    for i in range(pietra):
        do_otwarcia=os.path.join(nazwa, lista_plikow[i])
        print(do_otwarcia)
        plan=separator(read_file(do_otwarcia))
        plan = switch_object(plan)

        czysty_plan=copy.deepcopy(plan)
        #print_in_terminal(czysty_plan)

        plan=add_object(plan, tab_ludzie[i], CZLOWIEK)
        plan=add_object(plan, tab_przeszkody[i], PRZESZKODA)
        #print_in_terminal(plan)

        if i==0:
            tryb='w'
        else:
            tryb='a'
        save(nazwa_do_zapisu, pietra, tryb, i, tab_ludzie[i], tab_przeszkody[i], plan, czysty_plan)

main()
