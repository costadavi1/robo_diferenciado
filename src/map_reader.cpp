#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <filesystem>
#include <cctype>

class SimpleMapServer : public rclcpp::Node
{
public:
    SimpleMapServer() : Node("simple_map_server")
    {
        this->declare_parameter<std::string>("yaml_file", "map.yaml");
        std::string yaml_file = this->get_parameter("yaml_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Carregando YAML: %s", yaml_file.c_str());

        loadMapFromYaml(yaml_file);

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimpleMapServer::publishMap, this));
    }

private:
    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ----------------------------------------------------
    // Função segura que pula espaços, tabs, CR e comentários
    // ----------------------------------------------------
    void skipComments(std::ifstream &f)
    {
        while (true)
        {
            int c = f.peek();

            // fim do arquivo
            if (c == EOF)
                return;

            // comentário "# ..."
            if (c == '#')
            {
                std::string line;
                std::getline(f, line);
                continue;
            }

            // whitespace: espaço, enter, tab
            if (std::isspace(c))
            {
                f.get();
                continue;
            }

            // caso contrário, chegamos onde interessa
            break;
        }
    }

    // ----------------------------------------------------
    // Leitura de PGM correta
    // ----------------------------------------------------
    std::vector<uint8_t> loadPGM(const std::string &filename, int &width, int &height, int &maxval)
    {
        std::ifstream f(filename, std::ios::binary);
        if (!f)
            throw std::runtime_error("Erro ao abrir arquivo PGM: " + filename);

        std::string magic;
        f >> magic;

        if (magic != "P5")
            throw std::runtime_error("Formato inválido: esperado P5, obtido " + magic);

        f.get(); // consome o '\n' após o magic

        skipComments(f);

        f >> width >> height;
        f.get(); // consome '\n'

        skipComments(f);

        f >> maxval;
        f.get(); // consome '\n'

        if (width <= 0 || height <= 0)
            throw std::runtime_error("Dimensões inválidas no PGM");

        if (maxval != 255)
            RCLCPP_WARN(this->get_logger(), "Aviso: PGM com maxval != 255 (%d)", maxval);

        std::vector<uint8_t> data(width * height);

        f.read(reinterpret_cast<char *>(data.data()), data.size());

        if (!f)
            throw std::runtime_error("Erro ao ler dados do PGM");

        RCLCPP_INFO(this->get_logger(),
                    "PGM carregado com sucesso: %dx%d max=%d", width, height, maxval);

        return data;
    }

    // ----------------------------------------------------
    // Lê YAML e converte mapa
    // ----------------------------------------------------
    void loadMapFromYaml(const std::string &yaml_file)
    {
        YAML::Node cfg = YAML::LoadFile(yaml_file);

        double resolution = cfg["resolution"].as<double>();
        auto origin = cfg["origin"].as<std::vector<double>>();
        int negate = cfg["negate"].as<int>();
        double occ_th = cfg["occupied_thresh"].as<double>();
        double free_th = cfg["free_thresh"].as<double>();
        std::string image_name = cfg["image"].as<std::string>();

        // Resolve caminho absoluto da imagem
        std::filesystem::path yaml_path(yaml_file);
        std::filesystem::path yaml_dir = yaml_path.parent_path();
        std::string image_path = (yaml_dir / image_name).string();

        int width, height, maxval;

        RCLCPP_INFO(this->get_logger(), "Carregando PGM: %s", image_path.c_str());

        std::vector<uint8_t> pgm = loadPGM(image_path, width, height, maxval);

        // Preenche metadata
        map_.info.resolution = resolution;
        map_.info.width = width;
        map_.info.height = height;

        map_.info.origin.position.x = origin[0];
        map_.info.origin.position.y = origin[1];
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        map_.data.resize(width * height);

        // ----------------------------------------------------
        // Conversão igual ao map_server do ROS
        // ----------------------------------------------------
        for (int i = 0; i < width * height; i++)
        {
            double p = pgm[i] / 255.0;

            if (!negate)
                p = 1.0 - p;

            int8_t val = -1; // desconhecido

            if (p > occ_th)
                val = 100; // ocupado
            else if (p < free_th)
                val = 0; // livre

            map_.data[i] = val;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Mapa convertido: %d x %d", width, height);
    }

    // ----------------------------------------------------
    // Publicador
    // ----------------------------------------------------
    void publishMap()
    {
        map_.header.stamp = this->now();
        map_.header.frame_id = "map";
        pub_->publish(map_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapServer>());
    rclcpp::shutdown();
    return 0;
}