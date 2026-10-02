/*
 *  This file is a part of KNOSSOS.
 *
 *  (C) Copyright 2007-2018
 *  Max-Planck-Gesellschaft zur Foerderung der Wissenschaften e.V.
 *
 *  KNOSSOS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 of
 *  the License as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  For further information, visit https://knossos.app
 *  or contact knossosteam@gmail.com
 */

#pragma once

/* Turning a Neuroglancer precomputed volume into something KNOSSOS can load.
 *
 * Reads the volume's info and maps its resolution levels onto KNOSSOS magnifications, so
 * that the rest of the program sees an ordinary overlay layer.
 *
 * Only the info is read here. Where a chunk lives is worked out from its coordinate when
 * it is wanted — see Dataset::precomputedRequest() — because resolving every chunk's
 * position in advance was measured at over five minutes on one volume, for nothing: a
 * shard holds about one chunk there, so its index arrives with the chunk anyway. */

#include "dataset.h"
#include "segmentation/precomputed.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <algorithm>

namespace precomputedload {

inline precomputed::Encoding encodingFromName(const QString & name) {
    if (name == "raw") { return precomputed::Encoding::Raw; }
    if (name == "compressed_segmentation") { return precomputed::Encoding::CompressedSegmentation; }
    return precomputed::Encoding::Unsupported;
}

// Parses the info document. Returns false, with why, if it is not a volume we can read.
inline bool parseInfo(const QByteArray & json, Dataset::Precomputed & out, QString & why) {
    const auto document = QJsonDocument::fromJson(json);
    if (!document.isObject()) {
        why = QObject::tr("the volume's info is not valid JSON");
        return false;
    }
    const auto root = document.object();
    if (root["@type"].toString() != "neuroglancer_multiscale_volume") {
        why = QObject::tr("not a neuroglancer_multiscale_volume");
        return false;
    }
    const auto dataType = root["data_type"].toString();
    if (dataType != "uint64" && dataType != "uint32" && dataType != "uint16" && dataType != "uint8") {
        why = QObject::tr("a segmentation of type %1 is not supported").arg(dataType);
        return false;
    }
    out.numChannels = std::max(1, root["num_channels"].toInt(1));
    out.scales.clear();
    for (const auto value : root["scales"].toArray()) {
        const auto entry = value.toObject();
        precomputed::Scale scale;
        scale.key = entry["key"].toString().toStdString();
        const auto size = entry["size"].toArray();
        const auto offset = entry["voxel_offset"].toArray();
        const auto resolution = entry["resolution"].toArray();
        const auto chunks = entry["chunk_sizes"].toArray();
        if (size.size() != 3 || chunks.isEmpty() || chunks.first().toArray().size() != 3) {
            continue;// a level we cannot make sense of is skipped rather than guessed at
        }
        const auto chunk = chunks.first().toArray();
        for (int a = 0; a < 3; ++a) {
            scale.size[a] = static_cast<std::int64_t>(size.at(a).toDouble());
            scale.chunk[a] = static_cast<std::int32_t>(chunk.at(a).toDouble());
            scale.voxelOffset[a] = offset.size() == 3 ? static_cast<std::int64_t>(offset.at(a).toDouble()) : 0;
            scale.resolution[a] = resolution.size() == 3 ? resolution.at(a).toDouble() : 1.0;
        }
        scale.encoding = encodingFromName(entry["encoding"].toString());
        const auto csegBlock = entry["compressed_segmentation_block_size"].toArray();
        if (csegBlock.size() == 3) {
            for (int a = 0; a < 3; ++a) {
                scale.csegBlock[a] = static_cast<std::int32_t>(csegBlock.at(a).toDouble());
            }
        }
        if (entry.contains("sharding")) {
            const auto sharding = entry["sharding"].toObject();
            if (sharding["@type"].toString() != "neuroglancer_uint64_sharded_v1") {
                why = QObject::tr("sharding of type %1 is not supported").arg(sharding["@type"].toString());
                return false;
            }
            if (sharding["hash"].toString() != "identity") {
                why = QObject::tr("a %1 shard hash is not supported").arg(sharding["hash"].toString());
                return false;
            }
            scale.sharded = true;
            scale.preshiftBits = sharding["preshift_bits"].toInt();
            scale.minishardBits = sharding["minishard_bits"].toInt();
            scale.shardBits = sharding["shard_bits"].toInt();
            scale.shardedIndexGzip = sharding["minishard_index_encoding"].toString() == "gzip";
            scale.shardedDataGzip = sharding["data_encoding"].toString() == "gzip";
        }
        out.scales.push_back(scale);
    }
    if (out.scales.empty()) {
        why = QObject::tr("the volume declares no usable resolution levels");
        return false;
    }
    // finest first, so level 0 is magnification 1
    std::sort(std::begin(out.scales), std::end(out.scales), [](const precomputed::Scale & a, const precomputed::Scale & b) {
        return a.size[0] > b.size[0];
    });
    return true;
}


}
