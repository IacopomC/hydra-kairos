/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once

#include <kimera_pgmo/mesh_types.h>
#include <spark_dsg/mesh.h>

/**
 * @file pgmo_mesh_traits.h
 * @brief Free-function trait overloads that let the mesh-deformation library read and
 *        write a scene-graph mesh generically. They cover vertex and face counts,
 *        resizing, per-vertex position/color/timestamp/label access, and face access.
 *        Declared in the mesh's namespace so they are found by argument-dependent lookup.
 */

namespace spark_dsg {

/// @brief Number of vertices in the mesh.
size_t pgmoNumVertices(const Mesh& mesh);

/// @brief Resize the mesh vertex storage.
void pgmoResizeVertices(Mesh& mesh, size_t size);

/// @brief Read a vertex position and, if a traits pointer is given, fill in its color,
/// timestamp and label where the mesh provides them.
/// @param mesh Mesh to read from.
/// @param i Vertex index.
/// @param traits Optional output for color, stamp and label attributes.
/// @return The vertex position.
Eigen::Vector3f pgmoGetVertex(const Mesh& mesh,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits = nullptr);

/// @brief Write a vertex position and any provided attributes the mesh supports.
/// @param mesh Mesh to modify.
/// @param i Vertex index.
/// @param pos New vertex position.
/// @param traits Optional color, stamp and label values to apply.
void pgmoSetVertex(Mesh& mesh,
                   size_t i,
                   const Eigen::Vector3f& pos,
                   const kimera_pgmo::traits::VertexTraits& traits = {});

/// @brief Return the timestamp of a vertex.
/// @throws std::runtime_error if the mesh carries no timestamps.
uint64_t pgmoGetVertexStamp(const Mesh& mesh, size_t i);

/// @brief Number of faces in the mesh.
size_t pgmoNumFaces(const Mesh& mesh);

/// @brief Resize the mesh face storage.
void pgmoResizeFaces(Mesh& mesh, size_t size);

/// @brief Read the vertex indices of a face.
kimera_pgmo::traits::Face pgmoGetFace(const Mesh& mesh, size_t i);

/// @brief Write the vertex indices of a face.
void pgmoSetFace(Mesh& mesh, size_t i, const kimera_pgmo::traits::Face& face);

}  // namespace spark_dsg
