module YourName
  module MultiPushPull
    class MultiPushPullTool

      def create_geometry_follow(ents)
        affected_faces = []

        if @distance < 0
          cutting_edges = []
          sliding_edges = []
          
          @boundary_edges.each do |e|
            f_s = (e.faces & @selection).first
            f_u = (e.faces - @selection).first
            if edge_angle_less_than_180?(e, f_s, f_u)
              sliding_edges << e
            else
              cutting_edges << e
            end
          end

          cutting_verts = cutting_edges.map(&:vertices).flatten.uniq
          sliding_verts = sliding_edges.map(&:vertices).flatten.uniq
          verts_to_move = sliding_verts - cutting_verts

          vert_to_new_pos = {}
          vertices = @selection.map(&:vertices).flatten.uniq
          vertices.each do |v|
            v_base = @follow_vectors[v]
            vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
            vert_to_new_pos[v] = v.position.offset(vec)
          end
          
          cutting_edges.each do |edge|
            f_s = (edge.faces & @selection).first
            next unless f_s

            v1 = edge.reversed_in?(f_s) ? edge.end : edge.start
            v2 = edge.reversed_in?(f_s) ? edge.start : edge.end

            pt1 = v1.position
            pt2 = v2.position
            new_pt1 = vert_to_new_pos[v1]
            new_pt2 = vert_to_new_pos[v2]
            
            edge_vec = pt1.vector_to(pt2)
            outward_norm = edge_vec.cross(f_s.normal)
            outward_norm.reverse! 
            
            created_faces = create_side_faces(ents, pt1, pt2, new_pt2, new_pt1, outward_norm)
            affected_faces.concat(created_faces)
          end
          
          @selection.each do |face|
            new_pts = face.vertices.map { |v| vert_to_new_pos[v] }
            target_norm = @keep_original ? face.normal.reverse : face.normal
            caps = create_cap_faces(ents, face, new_pts, target_norm)
            affected_faces.concat(caps)
          end
          
          unless @keep_original
            faces_to_erase = @selection.select(&:valid?)
            ents.erase_entities(faces_to_erase) if faces_to_erase.any?
            
            edges_to_erase = @internal_edges.select(&:valid?)
            ents.erase_entities(edges_to_erase) if edges_to_erase.any?
          end
          
          if verts_to_move.any?
            valid_verts = []
            valid_vecs = []
            verts_to_move.each do |v|
              if v.valid?
                valid_verts << v
                v_base = @follow_vectors[v]
                valid_vecs << Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
              end
            end
            begin
              ents.transform_by_vectors(valid_verts, valid_vecs) if valid_verts.any?
              affected_faces.concat(valid_verts.flat_map(&:faces)) if valid_verts.any?
            rescue => e
            end
          end
          
          return affected_faces

        else
          @boundary_edges.each do |edge|
            face = (edge.faces & @selection).first
            next unless face
            
            v1 = edge.reversed_in?(face) ? edge.end : edge.start
            v2 = edge.reversed_in?(face) ? edge.start : edge.end
            
            pt1 = v1.position
            pt2 = v2.position
            
            v1_base = @follow_vectors[v1]
            vec1 = Geom::Vector3d.new(v1_base.x * @distance, v1_base.y * @distance, v1_base.z * @distance)
            
            v2_base = @follow_vectors[v2]
            vec2 = Geom::Vector3d.new(v2_base.x * @distance, v2_base.y * @distance, v2_base.z * @distance)
            
            new_pt1 = pt1.offset(vec1)
            new_pt2 = pt2.offset(vec2)
            
            edge_vec = pt1.vector_to(pt2)
            outward_norm = edge_vec.cross(face.normal)
            
            created_faces = create_side_faces(ents, pt1, pt2, new_pt2, new_pt1, outward_norm)
            affected_faces.concat(created_faces)
          end

          @selection.each do |face|
            new_pts = face.vertices.map do |v|
              v_base = @follow_vectors[v]
              vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
              v.position.offset(vec)
            end
            
            target_norm = face.normal
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
      end

      def draw_follow(view, edge_color, tr)
        @selection.each do |face|
          new_pts = face.vertices.map do |v|
            v_base = @follow_vectors[v]
            vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
            v.position.offset(vec).transform(tr)
          end
          view.line_width = 2
          view.drawing_color = edge_color
          view.draw(GL_LINE_LOOP, new_pts)
        end

        is_shrinking = (@distance < 0 && !@keep_original)

        if is_shrinking
          vertices = []
          @selection.each { |f| vertices.concat(f.vertices) }
          vertices.uniq.each do |v|
            pt1 = v.position.transform(tr)
            v_base = @follow_vectors[v]
            vec = Geom::Vector3d.new(v_base.x * @distance, v_base.y * @distance, v_base.z * @distance)
            pt2 = v.position.offset(vec).transform(tr)
            
            view.line_width = 1
            view.drawing_color = edge_color
            view.draw(GL_LINES, [pt1, pt2])
          end
        else
          @boundary_edges.each do |edge|
            pt1_local = edge.start.position
            pt2_local = edge.end.position
            
            pt1 = pt1_local.transform(tr)
            pt2 = pt2_local.transform(tr)
            
            v1_base = @follow_vectors[edge.start]
            vec1 = Geom::Vector3d.new(v1_base.x * @distance, v1_base.y * @distance, v1_base.z * @distance)
            
            v2_base = @follow_vectors[edge.end]
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
end