# czyta z pliku po linii
def read_file(nazwa):
    plik = open(nazwa, 'r')
    try:
        zawartosc = plik.read()
    finally:
        plik.close()
    return zawartosc


def give_name():
    nazwa_pliku = input("Podaj nazwe pliku: ")
    return nazwa_pliku


def map_to_vector(tab):
    tab = tab.splitlines()
    ilosc_pieter = int(tab[0])

    macierz = []
    mapa3D = []
    for i in range(1, len(tab)):
        if tab[i] != '':
            tab[i] = tab[i].split()

        if len(tab[i]) > 2:
            macierz.append(tab[i])

        if tab[i] == '' and len(macierz) > 0:
            mapa3D.append(macierz.copy())
            macierz.clear()

    mapa3D.append(macierz.copy())

    return mapa3D, macierz, tab


def save_map():
    pass


def status():
    pass


def print_map_terminal(tab):
    for i in range(len(tab)):
        print(tab[i])


def swap(a, b):
    a, b = b, a
    return a, b


d, m, r = map_to_vector(read_file(give_name()))  # zmienic później na wczytywanie róznych def give_name()

# print_map_terminal(d[1]) #[pietro] -wyswietla po jednym pietrze

print_map_terminal(d[1])  # [pietro] -wyswietla po jednym pietrze
