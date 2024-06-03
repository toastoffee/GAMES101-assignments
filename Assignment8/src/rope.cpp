#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL
{

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        //        Comment-in this part when you implement the constructor

        for (int i = 0; i < num_nodes; ++i)
        {
            Vector2D massPos = start + i * (end - start) / (num_nodes - 1);
            Mass *massNode = new Mass(massPos, node_mass, false);
            massNode->velocity = Vector2D(0, 0);
            masses.push_back(massNode);
        }

        for (int i = 1; i < num_nodes; ++i)
        {
            Spring *springConnect = new Spring(masses[i - 1], masses[i], k);
            springs.push_back(springConnect);
        }

        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D direction = s->m2->position - s->m1->position;
            double distance = direction.norm();

            // std::cout << std::abs(direction.x * direction.x) << std::endl;
            // std::cout << std::abs(direction.y * direction.y) << std::endl;
            // std::cout << distance << std::endl;

            Vector2D force = s->k * direction / distance * (distance - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;

                float dumpFactor = 5e-3;
                m->forces += -dumpFactor * m->velocity;

                Vector2D accelerate = m->forces / m->mass;

                //explicit Euler
                // m->position += m->velocity * delta_t;
                // m->velocity += accelerate * delta_t;

                //semi-implicit Euler
                m->velocity += accelerate * delta_t;
                m->position += m->velocity * delta_t;

                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dir = (s->m2->position - s->m1->position);
            double length = dir.norm();
            Vector2D mov = (length - s->rest_length) * dir.unit() / 2.;
            if (!s->m1->pinned)
            {
                s->m1->position += mov;
            }
            if (!s->m2->pinned)
            {
                s->m2->position -= mov;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // damping factor
                double kd = 5e-5;
                m->position = m->position + (1 - kd) * (m->position - m->last_position) + gravity * delta_t * delta_t;
                m->last_position = temp_position;
            }
        }
    }
}
