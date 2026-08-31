// Checks that a MaxId in a [[Layer]] table reads back as the parser in dataset.cpp reads it,
// including the "absent means 0" case that marks the field undeclared.
#include <toml.hpp>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

std::uint64_t maxIdOf(const toml::value & layer) {
    return static_cast<std::uint64_t>(toml::find_or<std::int64_t>(layer, "MaxId", 0));
}

int main() {
    const std::string doc = R"(
[[Layer]]
Name = "image"
Description = "em"

[[Layer]]
Name = "seg"
Description = "segmentation"
MaxId = 52310

[[Layer]]
Name = "big"
Description = "segmentation"
MaxId = 4294967296
)";
    const std::vector<unsigned char> blob{std::cbegin(doc), std::cend(doc)};
    const auto config = toml::parse(blob, "test.k.toml");
    const auto & layers = toml::find(config, "Layer").as_array();
    assert(layers.size() == 3);
    assert(maxIdOf(layers.at(0)) == 0);           // undeclared
    assert(maxIdOf(layers.at(1)) == 52310u);
    assert(maxIdOf(layers.at(2)) == 4294967296u); // past 32 bits, as ids can be
    std::puts("datasetmaxid_test ok");
    return 0;
}
