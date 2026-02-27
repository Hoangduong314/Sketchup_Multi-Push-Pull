module YourName
  module MultiPushPull
    class MultiPushPullTool

      def create_geometry_joint(ents)
        affected_faces = []
        
        @boundary_edges.each do |edge|
          face = (edge.faces & @selection).first
          next unless face
          
          v1 = edge.reversed_in?(face) ? edge.end : edge.start
          v2 = edge.reversed_in?(face) ? edge.start : edge.end
          
          pt1 = v1.position
          pt2 = v2.position
          
          v1_base = @vertex_vectors[v1]
          vec1 = Geom::Vector3d.new(v1_base.x * @distance, v1_base.y * @distance, v1_base.z * @distance)
          
          v2_base = @vertex_vectors[v2]
          vec2 = Geom::Vector3d.new(v2_base.x * @distance, v2_base.y * @distance, v2_base.z * @distance)
          
          new_pt1 = pt1.offset(vec1)
          new_pt2 = pt2.offset(vec2)
          
          edge_vec = pt1.vector_to(pt2)
          outward_norm = edge_vec.cross(face.normal)
          outward_norm.reverse! if @distance < 0
          
          created_faces = create_side_faces(ents, pt1, pt2, new_pt2, new_pt1, outward_norm)
          affected_faces.concat(created_faces)
        end

        @selection.each do |face|
          new_pts = face.vertices.map do |v|
            v_base = @vertex_vectors[v]
            vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
            v.position.offset(vec)
          end
          
          target_norm = (@distance < 0 && @keep_original) ? face.normal.reverse : face.normal
          caps = create_cap_faces(ents, face, new_pts, target_norm)
          affected_faces.concat(caps)
        end

        unless @keep_original
          faces_to_erase = @selection.select(&:valid?)
          ents.erase_entities(faces_to_erase) if faces_to_erase.any?

          edges_to_erase = @internal_edges.select(&:valid?)
          ents.erase_entities(edges_to_erase) if edges_to_erase.any?
        end

        return affected_faces
      end

      def draw_joint(view, edge_color, tr)
        @selection.each do |face|
          new_pts = face.vertices.map do |v|
            v_base = @vertex_vectors[v]
            vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
            v.position.offset(vec).transform(tr)
          end
          view.line_width = 2
          view.drawing_color = edge_color
          view.draw(GL_LINE_LOOP, new_pts)
        end

        @boundary_edges.each do |edge|
          pt1_local = edge.start.position
          pt2_local = edge.end.position
          
          pt1 = pt1_local.transform(tr)
          pt2 = pt2_local.transform(tr)
          
          v1_base = @vertex_vectors[edge.start]
          vec1 = Geom::Vector3d.new(v1_base.x * @distance, v1_base.y * @distance, v1_base.z * @distance)
          
          v2_base = @vertex_vectors[edge.end]
          vec2 = Geom::Vector3d.new(v2_base.x * @distance, v2_base.y * @distance, v2_base.z * @distance)
          
          new_pt1 = pt1_local.offset(vec1).transform(tr)
          new_pt2 = pt2_local.offset(vec2).transform(tr)

          view.line_width = 1
          view.drawing_color = edge_color
          view.draw(GL_LINE_LOOP, [pt1, pt2, new_pt2, new_pt1])
        end
      end

    end
  end
end