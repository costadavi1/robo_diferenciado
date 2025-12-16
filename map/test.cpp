#include <iostream>
#include <fstream>

auto skipComments = [&](std::ifstream &f)
{
    while (true)
    {
        int c = f.peek();
        if (c == '#')
        {
            std::string line;
            std::getline(f, line);  // descarta comentário inteiro
        }
        else if (std::isspace(c))
        {
            f.get(); // descarta espaços / quebras de linha
        }
        else
        {
            break;   // chegamos nas dimensões
        }
    }
};

int main() {
    std::ifstream f("map.pgm", std::ios::binary);
    if (!f) {
        std::cout << "ERRO: Não abriu o arquivo.\n";
        return 0;
    }

    std::string magic;
    f >> magic;
    std::cout << "MAGIC: " << magic << "\n";

    skipComments(f);
    // char c;
    // f.get(c);
    // while (c == '#') {
    //     std::string comment;
    //     std::getline(f, comment);
    //     f.get(c);
    // }
    // f.unget();

    int w, h, maxval;
    f >> w >> h >> maxval;
    std::cout << "W=" << w << " H=" << h << " MAX=" << maxval << "\n";

    return 0;
}